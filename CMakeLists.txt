cmake_minimum_required(VERSION 3.5)
project(ros_pt)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Add the directory containing your Python package
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install your Python scripts
install(PROGRAMS
  python/ros_driver.py  # Make sure to include the .py extension
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
