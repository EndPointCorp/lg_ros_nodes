cmake_minimum_required(VERSION 2.8.3)
project(lg_json_config)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lg_msg_defs
)

catkin_package(
  CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/serve_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

