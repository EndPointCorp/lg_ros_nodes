cmake_minimum_required(VERSION 2.8.3)
project(lg_sv)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lg_msg_defs
)

catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/launcher.py
  scripts/server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY webapps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/sv_server.test)
  add_rostest(test/sv_panoid_state.test)
endif()
