cmake_minimum_required(VERSION 2.8.3)
project(lg_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ProcessState.msg
  WindowGeometry.msg
)
generate_messages()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dev_webserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY webapps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
