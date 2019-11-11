cmake_minimum_required(VERSION 2.8.3)
project(state_proxy)

find_package(catkin REQUIRED COMPONENTS
  rospy lg_msg_defs
)

catkin_package(
  CATKIN_DEPENDS rospy lg_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/state_tracker.py
  scripts/state_tracker
  scripts/state_setter.py
  scripts/state_setter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
