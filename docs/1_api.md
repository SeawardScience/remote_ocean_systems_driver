# API Usage

This package provides functionalities for controlling a PT25 or R25 device and interfacing with it through ROS.

## Overview

The API consists of three main components:

1. **PT25 Driver**: A Python module (`pt25.py`) that provides a class for controlling the PT25 device over a serial connection.
2. **ROS Driver**: A ROS2 node (`ros_driver.py`) that interfaces with the PT25 device and publishes its position as a ROS topic.
3. **Command Sender**: A Python script (`command_sender.py`) that sends commands to the PT25 device.

## Usage

### PT25 Driver (pt25.py)
The pt25 class in pt25.py provides methods for controlling the PT25 device over a serial connection.

```python
# Initialize the PT25 device
pt25obj = pt25('/dev/ttyUSB0', 9600)

# Get settings for device A and B
pt25obj.get_settings('A')
pt25obj.get_settings('B')

# Poll the device for position
while True:
    pt25obj.poll('A')
    pt25obj.poll('B')
    time.sleep(1.0)
```

### ROS Driver (ros_driver.py)
The PT25ROS class in ros_driver.py is a ROS2 node that interfaces with the PT25 device and publishes its position as a ROS topic.

```python
from ros_pt.ros_driver import PT25ROS

# Initialize and spin the ROS node
def main(args=None):
    rclpy.init(args=args)
    pt25ros_obj = PT25ROS()
    rclpy.spin(pt25ros_obj)
    pt25ros_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

