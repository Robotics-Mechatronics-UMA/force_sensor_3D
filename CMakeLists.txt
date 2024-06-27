cmake_minimum_required(VERSION 2.8.3)
project(force_sensor_3D)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rosserial_python
  rosserial_arduino
  message_generation
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  tare_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs rosserial_python rosserial_arduino message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tare_client_node src/tare_client_node.cpp)

add_dependencies(tare_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(tare_client_node
  ${catkin_LIBRARIES}
)
