cmake_minimum_required(VERSION 2.8.3)
project(lg_navlib)

find_package(catkin REQUIRED COMPONENTS
  lg_msg_defs
  message_runtime
  rospy
  std_msgs
)

#catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/nav_mode
  scripts/navtransform
  scripts/nav_mode.py
  scripts/navtransform.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/test_nav_mode.test)
endif()
