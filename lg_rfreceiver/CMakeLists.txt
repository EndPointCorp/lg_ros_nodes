cmake_minimum_required(VERSION 2.8.3)
project(lg_rfreceiver)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs appctl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/sender.py
  scripts/actuator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
