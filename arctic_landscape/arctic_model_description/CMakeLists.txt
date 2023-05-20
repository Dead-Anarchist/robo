cmake_minimum_required(VERSION 3.0.4)
project(arctic_model_description)

find_package(catkin REQUIRED hector_sensors_description)

catkin_package()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )
