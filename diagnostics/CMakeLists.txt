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

# Add the test executable
#add_executable(battery_monitor_test
#  test/battery_level_monitor_test.cpp
#  src/sensors/battery_level_monitor.cpp
#  msg/diagnostics.pb.cc
#  msg/sensor_data.pb.cc
#)

# Add the test executable for Google Test
catkin_add_gtest(battery_monitor_test
  test/battery_level_monitor_test.cpp
  src/sensors/battery_level_monitor.cpp
  msg/diagnostics.pb.cc
  msg/sensor_data.pb.cc
)
catkin_add_gtest(estop_monitor_test
  test/emergency_stop_monitor_test.cpp
  src/sensors/emergency_stop_monitor.cpp
  msg/diagnostics.pb.cc
  msg/sensor_data.pb.cc
)

catkin_add_gtest(temperature_monitor_test
  test/temperature_level_monitor_test.cpp
  src/sensors/temperature_level_monitor.cpp
  msg/diagnostics.pb.cc
  msg/sensor_data.pb.cc
)

catkin_add_gtest(gps_accuracy_monitor_test
  test/gps_accuracy_monitor_test.cpp
  src/sensors/gps_accuracy_monitor.cpp
  msg/diagnostics.pb.cc
  msg/sensor_data.pb.cc
  src/utility/timer.cpp
)

catkin_add_gtest(internet_signal_monitor_test
  test/internet_signal_monitor_test.cpp
  src/sensors/internet_signal_monitor.cpp
  msg/diagnostics.pb.cc
  msg/sensor_data.pb.cc
  src/utility/timer.cpp
)

# Link the test executable with GTest, ROS, and Protobuf
target_link_libraries(battery_monitor_test
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GTEST_LIBRARIES}
  pthread  # GTest requires pthread for multithreading support
)

target_link_libraries(estop_monitor_test
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GTEST_LIBRARIES}
  pthread  # GTest requires pthread for multithreading support
)

target_link_libraries(temperature_monitor_test
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GTEST_LIBRARIES}
  pthread  # GTest requires pthread for multithreading support
)

target_link_libraries(gps_accuracy_monitor_test
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GTEST_LIBRARIES}
  pthread  # GTest requires pthread for multithreading support
)

target_link_libraries(internet_signal_monitor_test
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GTEST_LIBRARIES}
  pthread  # GTest requires pthread for multithreading support
)

# Enable CMake testing capabilities
enable_testing()

# Add the test to CMake
add_test(NAME BatteryMonitorTest COMMAND battery_monitor_test)

