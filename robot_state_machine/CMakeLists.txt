cmake_minimum_required(VERSION 3.0.2)
project(robot_state_machine)

## Compile as C++11, supported since ROS Kinetic
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Find the Boost libraries that we need (for statechart)
find_package(Boost REQUIRED COMPONENTS)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare the executable and link the necessary libraries
add_executable(robot_state_machine_node 
  src/robot_state_machine.cpp
  src/error.cpp 
  src/idle.cpp
  src/running.cpp
  src/route_planning.cpp
  src/move.cpp
  src/control.cpp
  src/behaviour_planning.cpp
  src/robot_sm_node.cpp)

## Link the Boost statechart library and ROS libraries to the executable
target_link_libraries(robot_state_machine_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executable for installation
install(TARGETS robot_state_machine_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
