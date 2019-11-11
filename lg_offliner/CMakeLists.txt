cmake_minimum_required(VERSION 2.8.3)
project(lg_offliner)

find_package(catkin REQUIRED COMPONENTS
  appctl
  lg_msg_defs
  rospy
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_msg_defs appctl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/lg_offliner_starter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_when_online.test)
  add_rostest(test/online/test_when_offline.test)
  catkin_add_nosetests(test/offline/test_basic.py)
endif()
