cmake_minimum_required(VERSION 2.8.3)
project(spacenav_remote)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/spacenav_remote.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
