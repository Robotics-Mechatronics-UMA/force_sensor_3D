cmake_minimum_required(VERSION 2.8.3)
project(3-axis_force_sensor_ESP32)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/force_ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
