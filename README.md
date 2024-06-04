# ros_pt

## About
This is a ROS2 package for the Remote Ocean Systems R25 tilt actuator. It also has tools for controlling a PT25 Pan & Tilt actuator. The device communicates over RS-422 however this is converted topside to RS-232 which the driver utilizes for communications. Because of the use of RS-422, the timing between characters is very important. 

## Installation
The package can be cloned via Git to the src directory in your ROS2 workspace. 

### Begin by getting the package the dependencies
You can use rosdep to get the neccesary dependencies for build and runtime. 

```
sudo rosdep init
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro <distro> -y
```

### Compile

Finally you can compile the package from the root directory of your ros workspace

```
colcon build
```

## Getting Started
After sourcing your ROS2 workspace in your terminal you can run the subatlantic node with the following command.

```ros2 run ros_pt ros_driver.py```

To move the tilt motor send a position goal in the form of a JointState position msg. The driver expects position in radians.

```ros2 topic pub cmd/addr_a sensor_msgs/msg/JointState "position: 2" --rate 20```

You can also use an example launch file to run the driver.

```ros2 launch ros_pt ros_pt.launch.xml```

Parameters can be adjusted from yaml files found in the config folder. Example config and launch files are included for testing.

## Additional Utilities
This driver also includes tools for running a Remote Ocean Systems pan and tilt. Another node could be written to accomplish this functionality. 

## Topics

### Subscribed Topics

- **`valve_msg`** (`hydraulic_interfaces/msg/Valve`): Subscribes to commands for individual valves.

### Published Topics

- **`serial_connection/to_device`** (`io_interfaces/msg/RawPacket`): Sends raw data packets to the IO interface.
- **`~/pos/addr_a`** (`sensor_msgs/JointState`): Publishes the current position of the PT25 device.
- **`~/cmd/addr_a`** (`sensor_msgs/JointState`): Publishes commands to control the PT25 device.

## Parameters

- **`port`** (str): The serial port to connect to (default: `/dev/ttyUSB0`).
- **`baudrate`** (int): The baudrate for serial communication (default: `9600`).
- **`poll_rate`** (float): Frequency of polling the device for its current position (default: `5.0 Hz`).
- **`min_cmd_delay`** (float): Minimum duration between consecutive commands in seconds (default: `1.0`).

- **`roll_topic`** (str): The topic to publish the roll (or pan) angle (default: `~/pos/addr_a`).
- **`roll_cmd_topic`** (str): The topic to subscribe for roll (or pan) angle commands (default: `~/cmd/addr_a`).
- **`roll_frame`** (str): The frame ID for the roll (or pan) angle (default: `pt_axis_a`).

## Credits 
This code was originally developed as a ROS1 driver for a Remote Ocean Systems Pan/Tilt motor by Dave Casagrande of the Roman Lab at URI. It has been modified to work with the Remote Ocean Systems R25 Tilt motor by Jake Bonney and Kris Krasnosky. 

## License 
This code is written under the Apache2.0 license. 
