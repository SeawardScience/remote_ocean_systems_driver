#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header

class TiltJoyToJoint(Node):
    """
    Map a single joystick axis ([-1..1]) to a position command by integrating
    a rate (rad/s). Publishes JointState for PT25ROS roll_cmd_topic.
    """
    def __init__(self):
        super().__init__('tilt_joy_to_joint')

        # --- Minimal params ---
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_topic', '~/cmd/addr_a')    # set to PT25ROS roll_cmd_topic
        self.declare_parameter('axis', 1)                      # joystick axis index (e.g., stick Y)
        self.declare_parameter('deadband', 0.05)               # ignore tiny noise
        self.declare_parameter('max_rate_deg_s', 60.0)         # speed at |axis|=1 (deg/s)
        self.declare_parameter('min_deg', 0.0)                 # soft lower limit (deg)
        self.declare_parameter('max_deg', 360.0)               # soft upper limit (deg)
        self.declare_parameter('hz', 10.0)                     # publish rate
        self.declare_parameter('joint_name', 'pt_axis_a')      # must match PT25ROS expects
        self.declare_parameter('initial_deg', 150.0)           # start position (deg)

        # --- Resolve ---
        self.joy_topic  = self.get_parameter('joy_topic').value
        self.cmd_topic  = self.get_parameter('cmd_topic').value
        self.axis_idx   = int(self.get_parameter('axis').value)
        self.deadband   = float(self.get_parameter('deadband').value)
        self.max_rate       = math.radians(float(self.get_parameter('max_rate_deg_s').value))  # rad/s
        self.min_pos    = math.radians(float(self.get_parameter('min_deg').value))
        self.max_pos    = math.radians(float(self.get_parameter('max_deg').value))
        self.dt         = 1.0 / float(self.get_parameter('hz').value)
        self.joint_name = self.get_parameter('joint_name').value
        self.pos        = math.radians(float(self.get_parameter('initial_deg').value))

        # --- State ---
        self.last_cmd = 0.0  # [-1..1]

        # --- ROS I/O ---
        self.sub = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
        self.pub = self.create_publisher(JointState, self.cmd_topic, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"tilt_joy_to_joint: axis={self.axis_idx} → {self.cmd_topic}, "
            f"rate={math.degrees(self.max_rate):.1f} deg/s, "
            f"limits=[{math.degrees(self.min_pos):.1f}, {math.degrees(self.max_pos):.1f}] deg"
        )

    def on_joy(self, msg: Joy):
        v = msg.axes[self.axis_idx] if 0 <= self.axis_idx < len(msg.axes) else 0.0
        if abs(v) < self.deadband:
            v = 0.0
        self.last_cmd = max(-1.0, min(1.0, v))

    def on_timer(self):
        # Integrate whenever there is a nonzero command
        if self.last_cmd != 0.0:
            self.pos += self.last_cmd * self.max_rate * self.dt
            # Clamp to soft limits (helps avoid spamming out-of-bounds on the PT25)
            self.pos = max(self.min_pos, min(self.max_pos, self.pos))

        # Publish JointState (PT25ROS expects radians and the same joint name)
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [self.pos]
        self.pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(TiltJoyToJoint())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
