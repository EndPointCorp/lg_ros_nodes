cmake_minimum_required(VERSION 2.8.3)
project(lg_wireless_devices)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lg_msg_defs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  lg_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/wireless_watcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
