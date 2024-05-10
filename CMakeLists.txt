cmake_minimum_required(VERSION 2.8.3)
project(3-axis_force_sensor_ESP32)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  rosserial_arduino
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs rosserial_arduino
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

