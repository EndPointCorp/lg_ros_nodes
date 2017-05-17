cmake_minimum_required(VERSION 2.8.3)
project(lg_twister)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  lg_common
  rospy
)

catkin_install_python(PROGRAMS
  scripts/mux_twists.py
  scripts/mux_twists
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
