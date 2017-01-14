cmake_minimum_required(VERSION 2.8.3)
project(rfreceiver)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/sender.py
  scripts/kill_browser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
