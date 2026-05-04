#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int32
from io_interfaces.msg import RawPacket
from remote_ocean_systems_driver import visca as protocol

qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability  = DurabilityPolicy.VOLATILE

COMMAND_INTERVAL_S = 0.12
FALLBACK_DEG = 180.0   # used for the other axis when its position is unknown yet


class AccuPositionerVisca(Node):
    def __init__(self):
        super().__init__('accu_positioner_visca')

        self.declare_parameter('visca_address',   1)
        self.declare_parameter('poll_rate',        5.0)
        self.declare_parameter('min_cmd_delay',    0.04)
        self.declare_parameter('default_speed',    6)
        self.declare_parameter('joy_topic',        '/joy')
        self.declare_parameter('pan.roll_frame',   'pt_axis_a')
        self.declare_parameter('pan.joy_axis',      0)
        self.declare_parameter('pan.joy_max_speed', 10)
        self.declare_parameter('pan.joy_deadband',  0.05)
        self.declare_parameter('tilt.roll_frame',  'pt_axis_b')
        self.declare_parameter('tilt.joy_axis',     -1)
        self.declare_parameter('tilt.joy_max_speed', 10)
        self.declare_parameter('tilt.joy_deadband',  0.05)

        self._addr          = self.get_parameter('visca_address').value
        poll_rate           = self.get_parameter('poll_rate').value
        min_cmd_delay       = self.get_parameter('min_cmd_delay').value
        self._default_spd   = self.get_parameter('default_speed').value
        joy_topic           = self.get_parameter('joy_topic').value
        self._pan_frame     = self.get_parameter('pan.roll_frame').value
        self._pan_joy_axis  = self.get_parameter('pan.joy_axis').value
        self._pan_joy_max   = self.get_parameter('pan.joy_max_speed').value
        self._pan_deadband  = self.get_parameter('pan.joy_deadband').value
        self._tilt_frame    = self.get_parameter('tilt.roll_frame').value
        self._tilt_joy_axis = self.get_parameter('tilt.joy_axis').value
        self._tilt_joy_max  = self.get_parameter('tilt.joy_max_speed').value
        self._tilt_deadband = self.get_parameter('tilt.joy_deadband').value

        self.min_cmd_duration = rclpy.duration.Duration(seconds=min_cmd_delay)

        # device state
        self.init_ready   = False
        self.pan_pos_deg  = None
        self.tilt_pos_deg = None
        self.pan_speed    = 0     # last commanded speed in Int32 ±80 range
        self.tilt_speed   = 0
        self.last_speed_cmd = self.get_clock().now()
        self.last_joy_pan   = 0
        self.last_joy_tilt  = 0

        # accumulate binary bytes; split on 0xFF into complete packets
        self._rx_buffer = bytearray()

        # command queue drained at COMMAND_INTERVAL_S
        self._cmd_queue = []
        self.create_timer(COMMAND_INTERVAL_S, self._drain_queue)

        # transport
        self._to_device_pub = self.create_publisher(RawPacket, '~/connection/to_device', 10)
        self.create_subscription(RawPacket, '~/connection/from_device', self.from_device_cb, 10)

        # position publishers — same topic names as Helios driver
        self._pan_pub  = self.create_publisher(JointState, '~/pos/addr_a', 10)
        self._tilt_pub = self.create_publisher(JointState, '~/pos/addr_b', 10)

        # command subscribers
        self.create_subscription(JointState, '~/cmd/addr_a',
                                 lambda m: self._roll_cmd_cb(m, 'pan'), qos)
        self.create_subscription(JointState, '~/cmd/addr_b',
                                 lambda m: self._roll_cmd_cb(m, 'tilt'), qos)
        self.create_subscription(Int32, '~/cmd_speed/addr_a',
                                 lambda m: self._speed_cmd_cb(m, 'pan'), qos)
        self.create_subscription(Int32, '~/cmd_speed/addr_b',
                                 lambda m: self._speed_cmd_cb(m, 'tilt'), qos)

        if self._pan_joy_axis >= 0 or self._tilt_joy_axis >= 0:
            self.create_subscription(Joy, joy_topic, self._joy_cb, 10)

        self.create_timer(1.0 / poll_rate, self._poll_callback)
        self.create_timer(1.0, self._settings_retry)

        self.get_logger().info(
            f'Querying Visca device at address {self._addr} (0x{0x80 | self._addr:02X})')
        self._send_bytes(protocol.encode_get_position(self._addr))

    # ------------------------------------------------------------------
    # Command queue
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
    # Receive — accumulate bytes, split on 0xFF
    # ------------------------------------------------------------------

    def from_device_cb(self, msg: RawPacket):
        try:
            raw = b''.join(msg.data)
        except Exception:
            return
        self._rx_buffer.extend(raw)
        while 0xFF in self._rx_buffer:
            idx = self._rx_buffer.index(0xFF)
            packet = bytes(self._rx_buffer[:idx + 1])
            self._rx_buffer = self._rx_buffer[idx + 1:]
            self._handle_packet(packet)

    def _handle_packet(self, data: bytes):
        result = protocol.decode_position(data, self._addr)
        if result is not None:
            self._handle_position(*result)
        # ACK (z0 4y ff) and completion (z0 5y ff) consumed silently

    def _handle_position(self, pan_deg: float, tilt_deg: float):
        if not self.init_ready:
            self.init_ready = True
            self.get_logger().info(
                f'Visca device ready — pan={pan_deg:.1f}° tilt={tilt_deg:.1f}°')

        self.pan_pos_deg  = pan_deg
        self.tilt_pos_deg = tilt_deg
        now = self.get_clock().now().to_msg()

        pan_msg = JointState()
        pan_msg.header.stamp    = now
        pan_msg.header.frame_id = self._pan_frame
        pan_msg.name            = [self._pan_frame]
        pan_msg.position        = [math.radians(pan_deg)]
        self._pan_pub.publish(pan_msg)

        tilt_msg = JointState()
        tilt_msg.header.stamp    = now
        tilt_msg.header.frame_id = self._tilt_frame
        tilt_msg.name            = [self._tilt_frame]
        tilt_msg.position        = [math.radians(tilt_deg)]
        self._tilt_pub.publish(tilt_msg)

    # ------------------------------------------------------------------
    # Poll — one request gives both axes
    # ------------------------------------------------------------------

    def _poll_callback(self):
        self._send_bytes(protocol.encode_get_position(self._addr))

    def _settings_retry(self):
        if not self.init_ready:
            self.get_logger().warn('Visca device not responding, retrying', once=True)
            self._send_bytes(protocol.encode_get_position(self._addr))

    # ------------------------------------------------------------------
    # Position command (absolute, radians)
    # ------------------------------------------------------------------

    def _roll_cmd_cb(self, msg: JointState, axis: str):
        if not self.init_ready:
            return
        deg = math.degrees(msg.position[0])
        if axis == 'pan':
            pan_deg  = deg
            tilt_deg = self.tilt_pos_deg if self.tilt_pos_deg is not None else FALLBACK_DEG
        else:
            pan_deg  = self.pan_pos_deg if self.pan_pos_deg is not None else FALLBACK_DEG
            tilt_deg = deg
        self._send_bytes(protocol.encode_cancel(self._addr))
        self._send_bytes(protocol.encode_drive_absolute(
            self._addr, pan_deg, tilt_deg, self._default_spd, self._default_spd))

    # ------------------------------------------------------------------
    # Speed command (signed Int32 ±80)
    # ------------------------------------------------------------------

    def _speed_cmd_cb(self, msg: Int32, axis: str):
        speed = max(-80, min(80, int(msg.data)))
        if axis == 'pan':
            self._apply_speed(speed, self.tilt_speed)
        else:
            self._apply_speed(self.pan_speed, speed)

    def _apply_speed(self, pan_speed: int, tilt_speed: int):
        if pan_speed == self.pan_speed and tilt_speed == self.tilt_speed:
            return

        now = self.get_clock().now()
        if (now - self.last_speed_cmd) < self.min_cmd_duration:
            self.get_logger().warn('min_cmd_delay not met, ignoring',
                                   throttle_duration_sec=1.0)
            return

        if pan_speed == 0 and tilt_speed == 0:
            self._send_bytes(protocol.encode_cancel(self._addr))
        else:
            pan_v  = _signed_visca(pan_speed)
            tilt_v = _signed_visca(tilt_speed)
            self._send_bytes(protocol.encode_drive_open_loop(self._addr, pan_v, tilt_v))

        self.pan_speed      = pan_speed
        self.tilt_speed     = tilt_speed
        self.last_speed_cmd = now

    # ------------------------------------------------------------------
    # Joystick
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy):
        new_pan  = self.pan_speed
        new_tilt = self.tilt_speed
        changed  = False

        if 0 <= self._pan_joy_axis < len(msg.axes):
            raw = msg.axes[self._pan_joy_axis]
            spd = int(round((raw if abs(raw) >= self._pan_deadband else 0.0) * self._pan_joy_max))
            if spd != self.last_joy_pan:
                self.last_joy_pan = spd
                new_pan = spd
                changed = True

        if 0 <= self._tilt_joy_axis < len(msg.axes):
            raw = msg.axes[self._tilt_joy_axis]
            spd = int(round((raw if abs(raw) >= self._tilt_deadband else 0.0) * self._tilt_joy_max))
            if spd != self.last_joy_tilt:
                self.last_joy_tilt = spd
                new_tilt = spd
                changed = True

        if changed:
            self._apply_speed(new_pan, new_tilt)


def _signed_visca(speed: int) -> int:
    if speed == 0:
        return 0
    v = protocol.int32_to_visca_speed(speed)
    return v if speed > 0 else -v


def main(args=None):
    rclpy.init(args=args)
    node = AccuPositionerVisca()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
