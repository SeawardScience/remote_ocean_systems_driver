#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pt25 import pt25
import math
import time


class PT25ROS(Node):
    def __init__(self):
        super().__init__('pt25')
        self.node_name = self.get_name()
        self.get_params()

        self.pt25 = pt25(self.port, self.baudrate)

        while self.pt25.get_settings('A') != 0:
            self.get_logger().warn('Unable to connect to ROS Pan/Tilt, Retrying every 1 sec', once=True)
            time.sleep(1.0)

        self.pt25.set_ccw_limit('A', self.pt25.settings['A']['factory_ccw_limit'])
        self.pt25.set_cw_limit('A', self.pt25.settings['A']['factory_cw_limit'])

        self.last_pitch_cmd = self.get_clock().now()
        self.last_roll_cmd = self.get_clock().now()
        self.min_cmd_duration = rclpy.duration.Duration(seconds=self.min_cmd_delay)

        self.init_subscribers()
        self.init_publishers()

        self.timer = self.create_timer(1.0 / self.poll_rate, self.poll_callback)

    def get_params(self):
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('poll_rate', 2.0)
        self.declare_parameter('min_cmd_delay', 1.0)
        self.declare_parameter('pitch_topic', 'pt25_pitch')
        self.declare_parameter('pitch_cmd_topic', 'pt25_pitch_cmd')
        self.declare_parameter('pitch_frame', 'pt25_pitch')
        self.declare_parameter('roll_topic', 'pt25_roll')
        self.declare_parameter('roll_cmd_topic', 'pt25_roll_cmd')
        self.declare_parameter('roll_frame', 'pt25_roll')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value
        self.min_cmd_delay = self.get_parameter('min_cmd_delay').get_parameter_value().double_value
        self.pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        self.pitch_cmd_topic = self.get_parameter('pitch_cmd_topic').get_parameter_value().string_value
        self.pitch_frame = self.get_parameter('pitch_frame').get_parameter_value().string_value
        self.roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        self.roll_cmd_topic = self.get_parameter('roll_cmd_topic').get_parameter_value().string_value
        self.roll_frame = self.get_parameter('roll_frame').get_parameter_value().string_value

    def init_subscribers(self):
        self.create_subscription(JointState, self.pitch_cmd_topic, self.pitch_cmd_cb, 10)
        self.create_subscription(JointState, self.roll_cmd_topic, self.roll_cmd_cb, 10)

    def init_publishers(self):
        self.pitch_pub = self.create_publisher(JointState, self.pitch_topic, 10)
        self.roll_pub = self.create_publisher(JointState, self.roll_topic, 10)

    def pitch_cmd_cb(self, msg):
        if msg.header.stamp.sec < (self.last_pitch_cmd + self.min_cmd_duration).seconds:
            self.get_logger().warn('Pitch command too soon after last, ignoring.')
            return
        elif msg.header.stamp.sec == (self.last_pitch_cmd + self.min_cmd_duration).seconds:
            if msg.header.stamp.nanosec < (self.last_pitch_cmd + self.min_cmd_duration).nanoseconds:
                self.get_logger().warn('Pitch command too soon after last, ignoring.')
                return
        self.last_pitch_cmd = msg.header.stamp
        self.pt25.set('B', msg.position[0] * 180. / math.pi)

    def roll_cmd_cb(self, msg):
        # if msg.header.stamp.sec < (self.last_roll_cmd + self.min_cmd_duration).seconds:
        #     self.get_logger().warn('Roll command too soon after last, ignoring.')
        #     return
        # elif msg.header.stamp.sec == (self.last_roll_cmd + self.min_cmd_duration).seconds:
        #     if msg.header.stamp.nanosec < (self.last_roll_cmd + self.min_cmd_duration).nanoseconds:
        #         self.get_logger().warn('Roll command too soon after last, ignoring.')
        #         return
        self.last_roll_cmd = msg.header.stamp
        self.pt25.set('A', msg.position[0] * 180. / math.pi)

    def poll_callback(self):
        self.poll_pitch()
        self.poll_roll()

    def poll_pitch(self):
        pass
        # pitch = self.pt25.poll('B')
        # if pitch < 0:
        #     self.get_logger().warn(f'Invalid pitch: {pitch:.3f}')
        #     return
        # pitch_msg = JointState()
        # pitch_msg.header.stamp = self.get_clock().now().to_msg()
        # pitch_msg.header.frame_id = self.pitch_frame
        # pitch_msg.name = [self.pitch_frame]
        # pitch_msg.position = [math.pi * pitch / 180]
        # pitch_msg.velocity = []
        # pitch_msg.effort = []
        # self.pitch_pub.publish(pitch_msg)

    def poll_roll(self):
        roll = self.pt25.poll('A')
        if roll < 0:
            self.get_logger().warn(f'Invalid roll: {roll:.3f}')
            return
        roll_msg = JointState()
        roll_msg.header.stamp = self.get_clock().now().to_msg()
        roll_msg.header.frame_id = self.roll_frame
        roll_msg.name = [self.roll_frame]
        roll_msg.position = [math.pi * roll / 180]
        roll_msg.velocity = []
        roll_msg.effort = []
        self.roll_pub.publish(roll_msg)


def main(args=None):
    rclpy.init(args=args)
    pt25ros_obj = PT25ROS()
    rclpy.spin(pt25ros_obj)
    pt25ros_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
