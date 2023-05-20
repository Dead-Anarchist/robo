cmake_minimum_required(VERSION 3.0.4)
project(basic_movements_actions)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES MoveDir.action
  MoveOn.action
  RotateDir.action
  RotateOn.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

catkin_install_python(PROGRAMS src/basic_movements_actions_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
