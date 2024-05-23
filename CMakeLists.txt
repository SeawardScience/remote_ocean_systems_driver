cmake_minimum_required(VERSION 3.5)
project(ros_pt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

# Install entry point scripts
install(PROGRAMS
  src/ros_driver.py
  DESTINATION lib/${PROJECT_NAME})

# Register the node with ament
ament_package()
