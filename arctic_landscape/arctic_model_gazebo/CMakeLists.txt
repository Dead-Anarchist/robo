cmake_minimum_required(VERSION 3.0.4)
project(arctic_model_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED
   COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  )

catkin_package()

# to print compiler warinings
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )
