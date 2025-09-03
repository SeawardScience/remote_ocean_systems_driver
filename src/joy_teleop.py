#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class TiltSpeedTeleop(Node):
    """
    Read one joystick axis (-1..+1) and publish a signed PT25 speed (-80..+80).
    Negative -> CCW, Positive -> CW, 0 -> stop.
    Publishes only when the computed speed changes.
    """
    def __init__(self):
        super().__init__('tilt_speed_teleop')

        # --- Params ---
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_speed_topic', '/pt25/cmd_speed')
        self.declare_parameter('axis', 7)
        self.declare_parameter('deadband', 0.05)
        self.declare_parameter('max_device_speed', 20)   # 0..80, each ~0.5 deg/s

        # --- Resolve ---
        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_speed_topic = self.get_parameter('cmd_speed_topic').value
        self.axis_idx = int(self.get_parameter('axis').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.max_speed_units = max(0, min(80, int(self.get_parameter('max_device_speed').value)))

        # --- State ---
        self.last_sent = None  # remembers last published speed

        # --- ROS I/O ---
        self.sub = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
        self.pub = self.create_publisher(Int32, self.cmd_speed_topic, 10)

        self.get_logger().info(
            f"[{self.get_name()}] listening on {self.joy_topic}, "
            f"axis={self.axis_idx} → {self.cmd_speed_topic}, "
            f"max_device_speed={self.max_speed_units} (units ~0.5 deg/s)"
        )

    def on_joy(self, msg: Joy):
        # Read selected axis
        v = msg.axes[self.axis_idx] if 0 <= self.axis_idx < len(msg.axes) else 0.0
        if abs(v) < self.deadband:
            v = 0.0

        # Map [-1..1] -> [-max..+max], clamp
        speed_units = int(round(v * self.max_speed_units))
        speed_units = max(-80, min(80, speed_units))

        # Only publish if value changed
        if self.last_sent is None or speed_units != self.last_sent:
            self.pub.publish(Int32(data=speed_units))
            self.last_sent = speed_units

def main():
    rclpy.init()
    rclpy.spin(TiltSpeedTeleop())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
