cmake_minimum_required(VERSION 2.8.3)
project(spacenav_remote)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rfid_scanner
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/spacenav_remote.py
  scripts/spacenav_remote
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
