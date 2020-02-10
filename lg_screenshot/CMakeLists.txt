cmake_minimum_required(VERSION 2.8.3)
project(lg_screenshot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lg_msg_defs
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS lg_msg_defs
)

catkin_install_python(PROGRAMS
  scripts/web_screenshot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY webapps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_screenshots.test)
endif()
