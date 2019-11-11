cmake_minimum_required(VERSION 2.8.3)
project(lg_lock)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  lg_msg_defs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_msg_defs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
