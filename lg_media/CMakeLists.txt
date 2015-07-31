cmake_minimum_required(VERSION 2.8.3)
project(lg_media)

find_package(catkin REQUIRED COMPONENTS
  lg_common
  message_generation
  rospy
)

catkin_python_setup()

add_message_files(
  FILES
  AdhocMedia.msg
  AdhocMedias.msg
)

add_service_files(
  FILES
  MediaAppsInfo.srv
)

generate_messages(
  DEPENDENCIES
  lg_common
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_common message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/client
  scripts/client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
