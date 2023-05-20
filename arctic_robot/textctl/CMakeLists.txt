cmake_minimum_required(VERSION 3.0.4)
project(textctl)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    std_msgs
  )

catkin_python_setup()  
  
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(textctl src/textctl.cpp)

add_dependencies(textctl ${catkin_EXPORTED_TARGETS})

target_link_libraries(textctl ${catkin_LIBRARIES})

if ("$ENV{ROS_DISTRO}" STREQUAL "noetic")
catkin_install_python(PROGRAMS
    scripts/textctl_client.py
    scripts/textctl_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

