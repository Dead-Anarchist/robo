cmake_minimum_required(VERSION 3.0.4)
project(test_robot_teleporter)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf
    std_msgs
    gazebo_msgs
    move_base_msgs
    geometry_msgs
    std_srvs
  )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

if ("$ENV{ROS_DISTRO}" STREQUAL "noetic")
catkin_install_python(PROGRAMS
    src/test_robot_teleporter_node.py
    tests/test_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
