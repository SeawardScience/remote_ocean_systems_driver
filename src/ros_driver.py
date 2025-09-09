#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from ros_pt.pt25 import pt25
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

## \brief A ROS node for controlling a PT25 device.
#
# This node interfaces with a PT25 device and publishes its position as a ROS topic.
class PT25ROS(Node):
    ## \brief Initializes the PT25ROS node.
    def __init__(self):
        super().__init__('pt25')
        self.node_name = self.get_name()
        self.get_params()
        self.init_subscribers()
        self.init_publishers()

        self.pt25 = pt25(self.port, self.baudrate)
        self.get_logger().info('Connecting to PT25 device at %s:%d' % (self.port, self.baudrate))

        while self.pt25.get_settings(self.address) != 0:
            self.get_logger().warn('Unable to connect to ROS Pan/Tilt, Retrying every 1 sec', once=True)
            time.sleep(1.0)

        self.get_logger().info('PT25 factory ccw limit %d' % (self.pt25.settings[self.address]['factory_ccw_limit']))
        self.get_logger().info('PT25 factory cw limit %d' % (self.pt25.settings[self.address]['factory_cw_limit']))

        # Set the ccw limit
        if self.ccw_limit != 0 and self.ccw_limit > self.pt25.settings[self.address]['factory_ccw_limit']:
            self.pt25.set_ccw_limit(self.address, self.ccw_limit)
            self.get_logger().info('PT25 ccw limit set to %d' % (self.ccw_limit))
        else:
            self.pt25.set_ccw_limit(self.address, self.pt25.settings[self.address]['factory_ccw_limit'])

        # Set the cw limit
        if self.cw_limit != 0 and self.cw_limit < self.pt25.settings[self.address]['factory_cw_limit']:
            self.pt25.set_cw_limit(self.address, self.cw_limit)
            self.get_logger().info('PT25 cw limit set to %d' % (self.cw_limit))
        else:
            self.pt25.set_cw_limit(self.address, self.pt25.settings[self.address]['factory_cw_limit'])
            self.get_logger().info('PT25 cw limit set to %d' % (self.pt25.settings[self.address]['factory_cw_limit']))
        
        self.pt25.get_settings(self.address)
        self.get_logger().info('PT25 user ccw limit set to %d' % (self.pt25.settings[self.address]['user_ccw_limit']))
        self.get_logger().info('PT25 user cw limit set to %d' % (self.pt25.settings[self.address]['user_cw_limit']))

        self.last_pitch_cmd = self.get_clock().now()
        self.last_roll_cmd = self.get_clock().now()
        self.min_cmd_duration = rclpy.duration.Duration(seconds=self.min_cmd_delay)

        self.timer = self.create_timer(1.0 / self.poll_rate, self.poll_callback)

    ## \brief Gets parameters from the ROS parameter server.
    def get_params(self):
        self.declare_parameter('port', '/dev/ttyS3')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('poll_rate', 5.0)
        self.declare_parameter('min_cmd_delay', 1.0)
        self.declare_parameter('address', 'A')
        self.declare_parameter('ccw_limit', 0)
        self.declare_parameter('cw_limit', 0)

        self.declare_parameter('roll_topic', '~/pos/addr_a')
        self.declare_parameter('roll_cmd_topic', '~/cmd/addr_a')
        self.declare_parameter('speed_cmd_topic', '~/cmd_speed/addr_a')
        self.declare_parameter('roll_frame', 'pt_axis_a')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value
        self.min_cmd_delay = self.get_parameter('min_cmd_delay').get_parameter_value().double_value
        self.address = self.get_parameter('address').get_parameter_value().string_value
        self.ccw_limit = self.get_parameter('ccw_limit').get_parameter_value().integer_value
        self.cw_limit = self.get_parameter('cw_limit').get_parameter_value().integer_value

        self.roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        self.roll_cmd_topic = self.get_parameter('roll_cmd_topic').get_parameter_value().string_value
        self.speed_cmd_topic = self.get_parameter('speed_cmd_topic').get_parameter_value().string_value
        self.roll_frame = self.get_parameter('roll_frame').get_parameter_value().string_value

    ## \brief Initializes the ROS subscribers.
    def init_subscribers(self):
        self.create_subscription(JointState, self.roll_cmd_topic, self.roll_cmd_cb, qos)
        self.create_subscription(Int32, self.speed_cmd_topic, self.speed_cmd_cb, qos)

    ## \brief Initializes the ROS publishers.
    def init_publishers(self):
        self.roll_pub = self.create_publisher(JointState, self.roll_topic, 10)

    ## \brief Callback for roll command messages.
    #  \param msg The incoming JointState message.
    def roll_cmd_cb(self, msg):
        self.last_roll_cmd = msg.header.stamp
        self.pt25.stop(self.address)
        self.pt25.set(self.address, msg.position[0] * 180. / math.pi)

    ## \brief Callback for speed command messages.Signed speed command in device units: [-80..80]. Negative -> CCW, Positive -> CW, 0 -> stop.
    #  \param msg The incoming Int32 message.
    def speed_cmd_cb(self, msg: Int32):
        speed = int(msg.data)
        if speed < -80: speed = -80
        if speed >  80: speed =  80
        # Call your new rotate helper (signed): <0 CCW, >0 CW, 0 stop
        self.pt25.rotate(self.address, speed)

    ## \brief Timer callback for polling the device.
    def poll_callback(self):
        self.poll(self.address)

    ## \brief Polls the device and publishes the position.
    #  \param address The address of the device.
    def poll(self, address):
        roll = self.pt25.poll(address)
        if roll < 0:
            self.get_logger().warn(f'Invalid position: {roll:.3f}')
            return
        roll_msg = JointState()
        roll_msg.header.stamp = self.get_clock().now().to_msg()
        roll_msg.header.frame_id = self.roll_frame
        roll_msg.name = [self.roll_frame]
        roll_msg.position = [math.pi * roll / 180]
        roll_msg.velocity = []
        roll_msg.effort = []
        self.roll_pub.publish(roll_msg)

## \brief Main function to initialize and spin the ROS node.
def main(args=None):
    rclpy.init(args=args)
    pt25ros_obj = PT25ROS()
    rclpy.spin(pt25ros_obj)
    pt25ros_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
