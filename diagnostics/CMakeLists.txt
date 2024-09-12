cmake_minimum_required(VERSION 3.0.2)
project(diagnostics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES sensor_diagnostics
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

add_executable(sensor_monitor_node
  src/main.cpp
  src/sensors/battery_level_monitor.cpp
  src/sensors/emergency_stop_monitor.cpp
  src/sensors/gps_accuracy_monitor.cpp
  src/sensors/internet_signal_monitor.cpp
  src/sensors/temperature_level_monitor.cpp
  src/middleware/ros_middleware.cpp
  src/middleware/ros2_middleware.cpp
  src/rti/real_time_scheduler.cpp
  src/utility/timer.cpp
  msg/diagnostics.pb.cc
  msg/sensor_data.pb.cc
)

# Link executable to dependencies
target_link_libraries(sensor_monitor_node
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# Add Protobuf-generated files
#add_custom_command(TARGET sensor_monitor_node
#  POST_BUILD
#  COMMAND ${PROTOBUF_COMPILER} --cpp_out=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/proto/diagnostics.proto
#  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/proto
#)
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

