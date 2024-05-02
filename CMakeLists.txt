cmake_minimum_required(VERSION 2.8.3)
project(3-axis_force_sensor_ESP32)

add_compile_options(-std=c++11)

set(${PROJECT_NAME}_DEPENDS
  #Package dependencies
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Create libraries

## Create executables (nodes)


