#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from io_interfaces.msg import RawPacket
import math
import diagnostic_updater
from remote_ocean_systems_driver import pt25 as protocol

qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability  = DurabilityPolicy.VOLATILE

# Minimum gap between commands sent to the device (matches original COMMAND_DELAY).
# Commands are queued and drained at this rate so the serial_connection's queue-
# depth-1 subscription never drops a message.
COMMAND_INTERVAL_S = 0.12


class PT25ROS(Node):
    def __init__(self):
        super().__init__('pt25')

        # ---- addresses list (declare first so per-address params can be forward-declared) ----
        self.declare_parameter('addresses', ['A', 'B'])
        addresses = self.get_parameter('addresses').value

        # ---- per-address params (forward-declared from the addresses list) ----
        self.addr_cfg = {}
        for addr in addresses:
            key = f'addr_{addr.lower()}'
            self.declare_parameter(f'{key}.enabled',        True)
            self.declare_parameter(f'{key}.ccw_limit',      0)
            self.declare_parameter(f'{key}.cw_limit',       0)
            self.declare_parameter(f'{key}.roll_frame',     f'pt_axis_{addr.lower()}')
            self.declare_parameter(f'{key}.joy_axis',       -1)
            self.declare_parameter(f'{key}.joy_max_speed',  10)
            self.declare_parameter(f'{key}.joy_deadband',   0.05)
            self.addr_cfg[addr] = {
                'enabled':       self.get_parameter(f'{key}.enabled').value,
                'ccw_limit':     self.get_parameter(f'{key}.ccw_limit').value,
                'cw_limit':      self.get_parameter(f'{key}.cw_limit').value,
                'roll_frame':    self.get_parameter(f'{key}.roll_frame').value,
                'joy_axis':      self.get_parameter(f'{key}.joy_axis').value,
                'joy_max_speed': self.get_parameter(f'{key}.joy_max_speed').value,
                'joy_deadband':  self.get_parameter(f'{key}.joy_deadband').value,
            }

        # ---- shared params ----
        self.declare_parameter('poll_rate',     5.0)
        self.declare_parameter('min_cmd_delay', 0.04)
        self.declare_parameter('joy_topic',     '/joy')
        poll_rate     = self.get_parameter('poll_rate').value
        min_cmd_delay = self.get_parameter('min_cmd_delay').value
        joy_topic     = self.get_parameter('joy_topic').value

        self.min_cmd_duration = rclpy.duration.Duration(seconds=min_cmd_delay)

        # ---- per-address runtime state ----
        self.enabled_addresses = [a for a in addresses if self.addr_cfg[a]['enabled']]
        self.settings       = {a: None  for a in self.enabled_addresses}
        self.settings_ready = {a: False for a in self.enabled_addresses}
        self.init_state     = {a: 0     for a in self.enabled_addresses}
        self.speed          = {a: -1    for a in self.enabled_addresses}
        self.last_speed_cmd = {a: self.get_clock().now() for a in self.enabled_addresses}
        self.last_rx_time   = {a: None  for a in self.enabled_addresses}
        self.poll_idx       = 0

        # ---- command queue: drained at COMMAND_INTERVAL_S to keep serial_connection happy ----
        self._cmd_queue = []
        self.create_timer(COMMAND_INTERVAL_S, self._drain_queue)

        # ---- raw transport ----
        self._to_device_pub = self.create_publisher(RawPacket, '~/connection/to_device', 10)
        self.create_subscription(RawPacket, '~/connection/from_device', self.from_device_cb, 10)

        # ---- per-address pubs/subs ----
        self.pos_pubs = {}
        for addr in self.enabled_addresses:
            tag = addr.lower()
            self.pos_pubs[addr] = self.create_publisher(JointState, f'~/pos/addr_{tag}', 10)
            self.create_subscription(JointState, f'~/cmd/addr_{tag}',
                                     lambda msg, a=addr: self.roll_cmd_cb(msg, a), qos)
            self.create_subscription(Int32, f'~/cmd_speed/addr_{tag}',
                                     lambda msg, a=addr: self.speed_cmd_cb(msg, a), qos)

        # ---- joystick (subscribe once if any axis is configured) ----
        if any(self.addr_cfg[a]['joy_axis'] >= 0 for a in self.enabled_addresses):
            self.create_subscription(Joy, joy_topic, self.joy_cb, 10)
        self.last_joy_speed = {a: 0 for a in self.enabled_addresses}

        # ---- timers ----
        self.create_timer(1.0 / poll_rate, self.poll_callback)
        self.create_timer(1.0, self.settings_retry)

        # ---- diagnostics ----
        self._diag = diagnostic_updater.Updater(self)
        self._diag.setHardwareID('ros_pt')
        self._diag.add('Comms', self._diag_comms)

        # ---- kick off initialization ----
        for addr in self.enabled_addresses:
            self.get_logger().info(f'Querying settings for address {addr}')
            self._send_bytes(protocol.encode_get_settings(addr))

    # ------------------------------------------------------------------
    # Command queue: one publish per COMMAND_INTERVAL_S tick
    # ------------------------------------------------------------------

    def _send_bytes(self, data: bytes):
        self._cmd_queue.append(data)

    def _drain_queue(self):
        if not self._cmd_queue:
            return
        data = self._cmd_queue.pop(0)
        pkt = RawPacket()
        pkt.header.stamp = self.get_clock().now().to_msg()
        pkt.data = [bytes([b]) for b in data]
        self._to_device_pub.publish(pkt)

    # ------------------------------------------------------------------
    # from_device callback — route by response prefix
    # ------------------------------------------------------------------

    def from_device_cb(self, msg: RawPacket):
        try:
            raw = b''.join(msg.data).decode('utf-8', errors='ignore').strip()
        except Exception:
            return
        if len(raw) < 2:
            return

        addr = raw[0]
        if addr not in self.enabled_addresses:
            return

        self.last_rx_time[addr] = self.get_clock().now()
        cmd_char = raw[1]
        if cmd_char == '?':
            self._handle_settings(addr, raw)
        elif cmd_char == 'f':
            self._handle_poll(addr, raw)
        # limit-set acks ('d', 'u', 's', 'p', '<', '>') consumed silently

    # ------------------------------------------------------------------
    # Settings initialization state machine
    # ------------------------------------------------------------------

    def _handle_settings(self, addr: str, data: str):
        s = protocol.decode_settings(data)
        if s is None:
            self.get_logger().warn(f'[{addr}] Failed to parse settings: {data!r}')
            return

        state = self.init_state[addr]

        if state == 0:
            self.settings[addr] = s
            self.get_logger().info(
                f'[{addr}] Settings: {s}')
            cfg = self.addr_cfg[addr]
            ccw = cfg['ccw_limit'] if (cfg['ccw_limit'] > 0 and
                                       cfg['ccw_limit'] > s['factory_ccw_limit']) \
                  else s['factory_ccw_limit']
            cw  = cfg['cw_limit']  if (cfg['cw_limit'] > 0 and
                                       cfg['cw_limit'] < s['factory_cw_limit']) \
                  else s['factory_cw_limit']
            self._send_bytes(protocol.encode_set_ccw_limit(addr, ccw))
            self._send_bytes(protocol.encode_set_cw_limit(addr, cw))
            self._send_bytes(protocol.encode_get_settings(addr))
            self.init_state[addr] = 1

        elif state == 1:
            self.settings[addr] = s
            self.get_logger().info(
                f'[{addr}] User limits: ccw={s["user_ccw_limit"]} cw={s["user_cw_limit"]}')
            self.settings_ready[addr] = True
            self.init_state[addr] = 2
            self.get_logger().info(f'[{addr}] Ready')

    def settings_retry(self):
        for addr in self.enabled_addresses:
            if not self.settings_ready[addr]:
                self.get_logger().warn(f'[{addr}] Settings not ready, retrying', once=True)
                self._send_bytes(protocol.encode_get_settings(addr))

    # ------------------------------------------------------------------
    # Poll — alternate one address per tick to avoid queue saturation
    # ------------------------------------------------------------------

    def poll_callback(self):
        ready = [a for a in self.enabled_addresses if self.settings_ready[a]]
        if not ready:
            return
        addr = ready[self.poll_idx % len(ready)]
        self.poll_idx += 1
        self._send_bytes(protocol.encode_poll(addr))

    def _handle_poll(self, addr: str, data: str):
        if not self.settings_ready[addr]:
            return
        degrees = protocol.decode_poll(data, addr, self.settings[addr])
        if degrees is None:
            self.get_logger().warn(f'[{addr}] Invalid poll response: {data!r}',
                                   throttle_duration_sec=2.0)
            return
        msg = JointState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.addr_cfg[addr]['roll_frame']
        msg.name            = [self.addr_cfg[addr]['roll_frame']]
        msg.position        = [math.pi * degrees / 180.0]
        msg.velocity        = []
        msg.effort          = []
        self.pos_pubs[addr].publish(msg)

    # ------------------------------------------------------------------
    # Position command (absolute, radians)
    # ------------------------------------------------------------------

    def roll_cmd_cb(self, msg: JointState, addr: str):
        if not self.settings_ready[addr]:
            return
        degrees = msg.position[0] * 180.0 / math.pi
        self._send_bytes(protocol.encode_stop(addr))
        cmd = protocol.encode_set(addr, degrees, self.settings[addr])
        if cmd is None:
            self.get_logger().warn(f'[{addr}] Position {degrees:.2f}° out of range')
            return
        self._send_bytes(cmd)

    # ------------------------------------------------------------------
    # Speed command (signed Int32, −80..80)
    # ------------------------------------------------------------------

    def speed_cmd_cb(self, msg: Int32, addr: str):
        if not self.settings_ready[addr]:
            return
        self._apply_speed(addr, int(msg.data))

    def _apply_speed(self, addr: str, speed: int):
        speed = max(-80, min(80, speed))
        now   = self.get_clock().now()

        if speed == 0:
            if self.speed[addr] != 0:
                self._send_bytes(protocol.encode_stop(addr))
                self.speed[addr] = 0
                self.last_speed_cmd[addr] = now
            return

        if speed != self.speed[addr] and \
                (now - self.last_speed_cmd[addr]) < self.min_cmd_duration:
            self.get_logger().warn(f'[{addr}] min_cmd_delay not met, ignoring',
                                   throttle_duration_sec=1.0)
            return

        spd = protocol.sanitize_speed(abs(speed))
        if speed < 0:
            self._send_bytes(protocol.encode_rotate_ccw(addr, spd))
        else:
            self._send_bytes(protocol.encode_rotate_cw(addr, spd))
        self.speed[addr] = speed
        self.last_speed_cmd[addr] = now

    # ------------------------------------------------------------------
    # Joystick
    # ------------------------------------------------------------------

    def joy_cb(self, msg: Joy):
        for addr in self.enabled_addresses:
            if not self.settings_ready[addr]:
                continue
            axis_idx = self.addr_cfg[addr]['joy_axis']
            if axis_idx < 0 or axis_idx >= len(msg.axes):
                continue
            raw      = msg.axes[axis_idx]
            deadband = self.addr_cfg[addr]['joy_deadband']
            max_spd  = self.addr_cfg[addr]['joy_max_speed']
            if abs(raw) < deadband:
                raw = 0.0
            speed = int(round(raw * max_spd))
            if speed == self.last_joy_speed[addr]:
                continue
            self.last_joy_speed[addr] = speed
            self._apply_speed(addr, speed)


    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _diag_comms(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        now = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=5.0)
        all_ok = True
        for addr in self.enabled_addresses:
            if self.last_rx_time[addr] is None:
                stat.add(f'addr_{addr}', 'no response')
                all_ok = False
            elif (now - self.last_rx_time[addr]) > timeout:
                elapsed = (now - self.last_rx_time[addr]).nanoseconds / 1e9
                stat.add(f'addr_{addr}', f'stale {elapsed:.1f}s')
                all_ok = False
            else:
                stat.add(f'addr_{addr}', 'ok')
        if all_ok:
            stat.summary(diagnostic_updater.DiagnosticStatus.OK, 'Communicating')
        else:
            stat.summary(diagnostic_updater.DiagnosticStatus.WARN, 'No device detected')
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = PT25ROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
