cmake_minimum_required(VERSION 3.0.4)
project(target_vehicle_description)

find_package(catkin)

catkin_package()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )
