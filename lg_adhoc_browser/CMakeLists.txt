cmake_minimum_required(VERSION 2.8.3)
project(lg_adhoc_browser)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lg_common
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  AdhocBrowser.msg
  AdhocBrowsers.msg
)

generate_messages(
  DEPENDENCIES
  lg_common
)


catkin_package(
  CATKIN_DEPENDS lg_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/client
  scripts/client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_client.py)
endif()

