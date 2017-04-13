cmake_minimum_required(VERSION 2.8.3)
project(lg_volume_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

#catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/volume_control.py
  scripts/volume_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
