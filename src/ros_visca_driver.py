#!/usr/bin/env python3
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int32, Bool
from io_interfaces.msg import RawPacket
import diagnostic_updater
import diagnostic_msgs.msg
from remote_ocean_systems_driver import visca as protocol

qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability  = DurabilityPolicy.VOLATILE

COMMAND_INTERVAL_S = 0.12
FALLBACK_DEG = 180.0


def _rising(buttons: list, last: list, idx: int) -> bool:
    if idx < 0 or idx >= len(buttons):
        return False
    return buttons[idx] == 1 and (idx >= len(last) or last[idx] == 0)


def _falling(buttons: list, last: list, idx: int) -> bool:
    if idx < 0 or idx >= len(last):
        return False
    return last[idx] == 1 and (idx >= len(buttons) or buttons[idx] == 0)


def _signed_visca(speed: int) -> int:
    if speed == 0:
        return 0
    v = protocol.int32_to_visca_speed(speed)
    return v if speed > 0 else -v


class AccuPositionerVisca(Node):
    def __init__(self):
        super().__init__('accu_positioner_visca')

        # ---- parameter declarations ----------------------------------------
        self.declare_parameter('visca_address',    1)
        self.declare_parameter('poll_rate',         5.0)
        self.declare_parameter('min_cmd_delay',     0.04)
        self.declare_parameter('default_speed',     6)
        self.declare_parameter('cmd_rate',          4.0)
        self.declare_parameter('joy_topic',         '/joy')

        self.declare_parameter('pan.roll_frame',    'pt_axis_a')
        self.declare_parameter('pan.joy_axis',       0)
        self.declare_parameter('pan.joy_max_speed',  10)
        self.declare_parameter('pan.joy_deadband',   0.05)

        self.declare_parameter('tilt.roll_frame',   'pt_axis_b')
        self.declare_parameter('tilt.joy_axis',      -1)
        self.declare_parameter('tilt.joy_max_speed', 10)
        self.declare_parameter('tilt.joy_deadband',  0.05)

        self.declare_parameter('home.joy_button',  -1)
        self.declare_parameter('home.pan_deg',      0.0)
        self.declare_parameter('home.tilt_deg',     0.0)
        self.declare_parameter('home.speed',        6)

        # limit buttons (match PanTiltControl.qml layout)
        self.declare_parameter('limits.pan_cw_btn',    1)
        self.declare_parameter('limits.pan_ccw_btn',   2)
        self.declare_parameter('limits.tilt_down_btn', 3)
        self.declare_parameter('limits.tilt_up_btn',   4)
        self.declare_parameter('limits.clear_btn',     5)
        # limit positions (-1.0 = read from device, do not override on startup)
        self.declare_parameter('limits.pan_cw_deg',    -1.0)
        self.declare_parameter('limits.pan_ccw_deg',   -1.0)
        self.declare_parameter('limits.tilt_up_deg',   -1.0)
        self.declare_parameter('limits.tilt_down_deg', -1.0)

        # ---- read parameters -----------------------------------------------
        self._addr         = self.get_parameter('visca_address').value
        poll_rate          = self.get_parameter('poll_rate').value
        min_cmd_delay      = self.get_parameter('min_cmd_delay').value
        self._default_spd  = self.get_parameter('default_speed').value
        cmd_rate           = self.get_parameter('cmd_rate').value
        joy_topic          = self.get_parameter('joy_topic').value

        self._pan_frame     = self.get_parameter('pan.roll_frame').value
        self._pan_joy_axis  = self.get_parameter('pan.joy_axis').value
        self._pan_joy_max   = self.get_parameter('pan.joy_max_speed').value
        self._pan_deadband  = self.get_parameter('pan.joy_deadband').value

        self._tilt_frame     = self.get_parameter('tilt.roll_frame').value
        self._tilt_joy_axis  = self.get_parameter('tilt.joy_axis').value
        self._tilt_joy_max   = self.get_parameter('tilt.joy_max_speed').value
        self._tilt_deadband  = self.get_parameter('tilt.joy_deadband').value

        self._home_joy_button = self.get_parameter('home.joy_button').value
        self._home_pan_deg    = self.get_parameter('home.pan_deg').value
        self._home_tilt_deg   = self.get_parameter('home.tilt_deg').value
        self._home_speed      = self.get_parameter('home.speed').value

        self._lim_pan_cw_btn    = self.get_parameter('limits.pan_cw_btn').value
        self._lim_pan_ccw_btn   = self.get_parameter('limits.pan_ccw_btn').value
        self._lim_tilt_down_btn = self.get_parameter('limits.tilt_down_btn').value
        self._lim_tilt_up_btn   = self.get_parameter('limits.tilt_up_btn').value
        self._lim_clear_btn     = self.get_parameter('limits.clear_btn').value

        self.min_cmd_duration = rclpy.duration.Duration(seconds=min_cmd_delay)

        # ---- device state --------------------------------------------------
        self.init_ready   = False
        self.pan_pos_deg  = None
        self.tilt_pos_deg = None
        self.pan_speed    = 0
        self.tilt_speed   = 0
        self.last_speed_cmd    = self.get_clock().now()
        self.last_joy_pan      = 0
        self.last_joy_tilt     = 0
        self._last_joy_buttons = []

        self._pending_combined_cmd = None   # (pan_deg, tilt_deg) or None

        # limit state
        self._lim_pan_cw       = None   # degrees; None = unknown
        self._lim_pan_ccw      = None
        self._lim_tilt_up      = None
        self._lim_tilt_down    = None
        self._limits_cleared   = False
        self._updating_from_device = False

        # ---- queues --------------------------------------------------------
        self._rx_buffer    = bytearray()
        self._cmd_queue    = []
        self._last_rx_time = None
        # tracks inquiry type for each pending response: 'position'|'limit_up'|'limit_down'
        self._inquiry_queue = deque()

        self.create_timer(COMMAND_INTERVAL_S, self._drain_queue)
        self.create_timer(1.0 / cmd_rate, self._combined_cmd_dispatch)

        # ---- transport -----------------------------------------------------
        self._to_device_pub = self.create_publisher(RawPacket, '~/connection/to_device', 10)
        self.create_subscription(RawPacket, '~/connection/from_device', self.from_device_cb, 10)

        # ---- position publishers -------------------------------------------
        self._pan_pub  = self.create_publisher(JointState, '~/pos/addr_a', 10)
        self._tilt_pub = self.create_publisher(JointState, '~/pos/addr_b', 10)


        # ---- command subscribers -------------------------------------------
        self.create_subscription(JointState, '~/cmd',
                                 self._combined_cmd_cb, qos)
        self.create_subscription(JointState, '~/cmd/addr_a',
                                 lambda m: self._roll_cmd_cb(m, 'pan'), qos)
        self.create_subscription(JointState, '~/cmd/addr_b',
                                 lambda m: self._roll_cmd_cb(m, 'tilt'), qos)
        self.create_subscription(Int32, '~/cmd_speed/addr_a',
                                 lambda m: self._speed_cmd_cb(m, 'pan'), qos)
        self.create_subscription(Int32, '~/cmd_speed/addr_b',
                                 lambda m: self._speed_cmd_cb(m, 'tilt'), qos)
        self.create_subscription(Bool, '~/cmd/go_home',
                                 lambda m: self._go_home() if m.data else None, qos)

        self.add_on_set_parameters_callback(self._on_parameters_changed)

        has_limit_btns = any(b >= 0 for b in [
            self._lim_pan_cw_btn, self._lim_pan_ccw_btn,
            self._lim_tilt_down_btn, self._lim_tilt_up_btn, self._lim_clear_btn])
        if joy_topic and (self._pan_joy_axis >= 0 or self._tilt_joy_axis >= 0 \
                or self._home_joy_button >= 0 or has_limit_btns):
            self.create_subscription(Joy, joy_topic, self._joy_cb, 10)

        self.create_timer(1.0 / poll_rate, self._poll_callback)
        self.create_timer(1.0, self._settings_retry)

        self._diag = diagnostic_updater.Updater(self)
        self._diag.setHardwareID(f'accu_positioner_visca addr={self._addr}')
        self._diag.add('Comms', self._diag_comms)
        self._diag.add('Position Limits', self._limits_diagnostic)

        self.get_logger().info(
            f'Querying Visca device at address {self._addr} (0x{0x80 | self._addr:02X})')
        self._send_inquiry(protocol.encode_get_position(self._addr), 'position')
        self._send_inquiry(protocol.encode_get_position_limit(self._addr, 1), 'limit_up')
        self._send_inquiry(protocol.encode_get_position_limit(self._addr, 0), 'limit_down')

    # ------------------------------------------------------------------
    # Command queue
    # ------------------------------------------------------------------

    def _send_bytes(self, data: bytes):
        self._cmd_queue.append(data)

    def _send_inquiry(self, data: bytes, inquiry_type: str):
        self._cmd_queue.append(data)
        self._inquiry_queue.append(inquiry_type)

    def _drain_queue(self):
        if not self._cmd_queue:
            return
        data = self._cmd_queue.pop(0)
        pkt = RawPacket()
        pkt.header.stamp = self.get_clock().now().to_msg()
        pkt.data = [bytes([b]) for b in data]
        self._to_device_pub.publish(pkt)

    # ------------------------------------------------------------------
    # Receive
    # ------------------------------------------------------------------

    def from_device_cb(self, msg: RawPacket):
        try:
            raw = b''.join(msg.data)
        except Exception:
            return
        self._last_rx_time = self.get_clock().now()
        self._rx_buffer.extend(raw)
        while 0xFF in self._rx_buffer:
            idx = self._rx_buffer.index(0xFF)
            packet = bytes(self._rx_buffer[:idx + 1])
            self._rx_buffer = self._rx_buffer[idx + 1:]
            self._handle_packet(packet)

    def _handle_packet(self, data: bytes):
        result = protocol.decode_position(data, self._addr)
        if result is not None:
            if not self._inquiry_queue:
                return
            itype = self._inquiry_queue.popleft()
            if itype == 'position':
                self._handle_position(*result)
            elif itype == 'limit_up':
                self._handle_limit_response(1, *result)
            elif itype == 'limit_down':
                self._handle_limit_response(0, *result)
        # ACK (z0 4y ff) and completion (z0 5y ff) consumed silently

    # ------------------------------------------------------------------
    # Position feedback
    # ------------------------------------------------------------------

    def _handle_position(self, pan_deg: float, tilt_deg: float):
        if not self.init_ready:
            self.init_ready = True
            self.get_logger().info(
                f'Visca device ready — pan={pan_deg:.1f}° tilt={tilt_deg:.1f}°')
            self._push_yaml_limits_if_configured()

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
    # Poll
    # ------------------------------------------------------------------

    def _poll_callback(self):
        self._send_inquiry(protocol.encode_get_position(self._addr), 'position')

    def _settings_retry(self):
        if not self.init_ready:
            self.get_logger().warn('Visca device not responding, retrying', once=True)
            self._send_inquiry(protocol.encode_get_position(self._addr), 'position')

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

    def _combined_cmd_cb(self, msg: JointState):
        if not self.init_ready:
            return
        if len(msg.position) < 2:
            return
        self._pending_combined_cmd = (
            math.degrees(msg.position[0]),
            math.degrees(msg.position[1]),
        )

    def _combined_cmd_dispatch(self):
        if self._pending_combined_cmd is None:
            return
        pan_deg, tilt_deg = self._pending_combined_cmd
        self._pending_combined_cmd = None
        self._cmd_queue.clear()
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
            self._send_bytes(protocol.encode_drive_open_loop(
                self._addr, _signed_visca(pan_speed), _signed_visca(tilt_speed)))
        self.pan_speed      = pan_speed
        self.tilt_speed     = tilt_speed
        self.last_speed_cmd = now

    # ------------------------------------------------------------------
    # Home
    # ------------------------------------------------------------------

    def _go_home(self):
        if not self.init_ready:
            return
        self._send_bytes(protocol.encode_cancel(self._addr))
        self._send_bytes(protocol.encode_drive_absolute(
            self._addr, self._home_pan_deg, self._home_tilt_deg,
            self._home_speed, self._home_speed))

    # ------------------------------------------------------------------
    # Limits — device interaction
    # ------------------------------------------------------------------

    def _handle_limit_response(self, direction: int, pan_deg: float, tilt_deg: float):
        label = 'up/right' if direction == 1 else 'down/left'
        # self.get_logger().info(
        #     f'Device limits {label}: pan={pan_deg:.1f}° tilt={tilt_deg:.1f}°')
        self._updating_from_device = True
        try:
            updates = []
            if direction == 1:
                if self.get_parameter('limits.pan_cw_deg').value < 0:
                    self._lim_pan_cw = pan_deg
                    updates.append(Parameter('limits.pan_cw_deg',
                                             Parameter.Type.DOUBLE, pan_deg))
                if self.get_parameter('limits.tilt_up_deg').value < 0:
                    self._lim_tilt_up = tilt_deg
                    updates.append(Parameter('limits.tilt_up_deg',
                                             Parameter.Type.DOUBLE, tilt_deg))
            else:
                if self.get_parameter('limits.pan_ccw_deg').value < 0:
                    self._lim_pan_ccw = pan_deg
                    updates.append(Parameter('limits.pan_ccw_deg',
                                             Parameter.Type.DOUBLE, pan_deg))
                if self.get_parameter('limits.tilt_down_deg').value < 0:
                    self._lim_tilt_down = tilt_deg
                    updates.append(Parameter('limits.tilt_down_deg',
                                             Parameter.Type.DOUBLE, tilt_deg))
            if updates:
                self.set_parameters(updates)
        finally:
            self._updating_from_device = False

    def _push_limit(self, direction: int):
        if direction == 1:
            pan  = self._lim_pan_cw   if self._lim_pan_cw  is not None else 360.0
            tilt = self._lim_tilt_up  if self._lim_tilt_up is not None else 360.0
        else:
            pan  = self._lim_pan_ccw   if self._lim_pan_ccw   is not None else 0.0
            tilt = self._lim_tilt_down if self._lim_tilt_down is not None else 0.0
        self._send_bytes(
            protocol.encode_set_position_limit(self._addr, direction, pan, tilt))

    def _push_yaml_limits_if_configured(self):
        p    = self.get_parameter
        cw   = p('limits.pan_cw_deg').value
        up   = p('limits.tilt_up_deg').value
        ccw  = p('limits.pan_ccw_deg').value
        down = p('limits.tilt_down_deg').value
        if cw >= 0 or up >= 0:
            self._lim_pan_cw  = cw  if cw  >= 0 else 360.0
            self._lim_tilt_up = up  if up  >= 0 else 360.0
            self._push_limit(1)
        if ccw >= 0 or down >= 0:
            self._lim_pan_ccw   = ccw  if ccw  >= 0 else 0.0
            self._lim_tilt_down = down if down >= 0 else 0.0
            self._push_limit(0)

    def _set_limit_at_current(self, which: str):
        if not self.init_ready:
            return
        val = self.pan_pos_deg if 'pan' in which else self.tilt_pos_deg
        if val is None:
            return
        attr_map  = {'pan_cw': '_lim_pan_cw', 'pan_ccw': '_lim_pan_ccw',
                     'tilt_up': '_lim_tilt_up', 'tilt_down': '_lim_tilt_down'}
        pname_map = {'pan_cw': 'limits.pan_cw_deg', 'pan_ccw': 'limits.pan_ccw_deg',
                     'tilt_up': 'limits.tilt_up_deg', 'tilt_down': 'limits.tilt_down_deg'}
        setattr(self, attr_map[which], val)
        self.set_parameters([Parameter(pname_map[which], Parameter.Type.DOUBLE, val)])

    def _clear_limits(self):
        if not self.init_ready:
            return
        self._send_bytes(protocol.encode_clear_position_limits(self._addr))
        self._limits_cleared = True
        self.get_logger().info('Position limits cleared')
        self._diag.force_update()

    def _restore_limits(self):
        if not self.init_ready or not self._limits_cleared:
            return
        if self._lim_pan_cw is not None or self._lim_tilt_up is not None:
            self._push_limit(1)
        if self._lim_pan_ccw is not None or self._lim_tilt_down is not None:
            self._push_limit(0)
        self._limits_cleared = False
        self.get_logger().info('Position limits restored')
        self._diag.force_update()

    def _diag_comms(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        now = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=5.0)
        if self._last_rx_time is None:
            stat.summary(diagnostic_updater.DiagnosticStatus.WARN, 'No device detected')
        elif (now - self._last_rx_time) > timeout:
            elapsed = (now - self._last_rx_time).nanoseconds / 1e9
            stat.summary(diagnostic_updater.DiagnosticStatus.WARN,
                         f'No device detected — last rx {elapsed:.1f}s ago')
        else:
            stat.summary(diagnostic_updater.DiagnosticStatus.OK, 'Communicating')
        return stat

    def _limits_diagnostic(self, stat):
        if not self.init_ready:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, 'No device connection')
        elif self._limits_cleared:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Position limits cleared')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Limits active')
        def _fmt(v): return f'{v:.1f}°' if v is not None else 'unknown'
        stat.add('pan_cw_deg',    _fmt(self._lim_pan_cw))
        stat.add('pan_ccw_deg',   _fmt(self._lim_pan_ccw))
        stat.add('tilt_up_deg',   _fmt(self._lim_tilt_up))
        stat.add('tilt_down_deg', _fmt(self._lim_tilt_down))
        return stat

    # ------------------------------------------------------------------
    # Parameter changes
    # ------------------------------------------------------------------

    _LIM_PARAM_MAP = {
        'limits.pan_cw_deg':    (1, '_lim_pan_cw'),
        'limits.tilt_up_deg':   (1, '_lim_tilt_up'),
        'limits.pan_ccw_deg':   (0, '_lim_pan_ccw'),
        'limits.tilt_down_deg': (0, '_lim_tilt_down'),
    }

    def _on_parameters_changed(self, params):
        push_dirs = set()
        for p in params:
            if p.name == 'home.pan_deg':
                self._home_pan_deg = p.value
            elif p.name == 'home.tilt_deg':
                self._home_tilt_deg = p.value
            elif p.name == 'home.speed':
                self._home_speed = p.value
            elif p.name == 'home.joy_button':
                self._home_joy_button = p.value
            elif p.name in self._LIM_PARAM_MAP and p.value >= 0.0:
                direction, attr = self._LIM_PARAM_MAP[p.name]
                setattr(self, attr, p.value)
                if not self._updating_from_device and self.init_ready:
                    push_dirs.add(direction)
        for d in push_dirs:
            self._push_limit(d)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Joystick
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)

        # home button
        if _rising(buttons, self._last_joy_buttons, self._home_joy_button):
            self._go_home()

        # limit buttons
        limit_map = {
            self._lim_pan_cw_btn:    'pan_cw',
            self._lim_pan_ccw_btn:   'pan_ccw',
            self._lim_tilt_down_btn: 'tilt_down',
            self._lim_tilt_up_btn:   'tilt_up',
        }
        for btn, which in limit_map.items():
            if _rising(buttons, self._last_joy_buttons, btn):
                self._set_limit_at_current(which)
        if _rising(buttons, self._last_joy_buttons, self._lim_clear_btn):
            self._clear_limits()
        elif _falling(buttons, self._last_joy_buttons, self._lim_clear_btn):
            self._restore_limits()

        self._last_joy_buttons = buttons

        new_pan  = self.pan_speed
        new_tilt = self.tilt_speed

        if 0 <= self._pan_joy_axis < len(msg.axes):
            raw = msg.axes[self._pan_joy_axis]
            new_pan = int(round((raw if abs(raw) >= self._pan_deadband else 0.0) * self._pan_joy_max))
            self.last_joy_pan = new_pan

        if 0 <= self._tilt_joy_axis < len(msg.axes):
            raw = msg.axes[self._tilt_joy_axis]
            new_tilt = int(round((raw if abs(raw) >= self._tilt_deadband else 0.0) * self._tilt_joy_max))
            self.last_joy_tilt = new_tilt

        self._apply_speed(new_pan, new_tilt)


def main(args=None):
    rclpy.init(args=args)
    node = AccuPositionerVisca()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
