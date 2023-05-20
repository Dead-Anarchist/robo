cmake_minimum_required(VERSION 3.0.2)
project(action_follow)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  tf  
  actionlib_msgs
  )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   FollowTF.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES parade_config
  CATKIN_DEPENDS actionlib_msgs
#  DEPENDS extended_object_detection
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

if ("$ENV{ROS_DISTRO}" STREQUAL "noetic")
catkin_install_python(PROGRAMS
  src/action_follow_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()
