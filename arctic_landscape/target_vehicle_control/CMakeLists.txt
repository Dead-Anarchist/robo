cmake_minimum_required(VERSION 3.0.4)
project(target_vehicle_control)

find_package(catkin REQUIRED tf std_srvs signproc_soar scenario_executioner)

catkin_package()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )

catkin_install_python(PROGRAMS scripts/arctic_demo_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
