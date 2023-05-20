cmake_minimum_required(VERSION 3.0.4)
project(arctic_manip_service)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  tf
)

add_service_files(
  FILES ArcticManipMoveTo.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS src/arctic_manip_service_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
