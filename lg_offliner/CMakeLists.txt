cmake_minimum_required(VERSION 2.8.3)
project(lg_offliner)

find_package(catkin REQUIRED COMPONENTS
  appctl
  lg_common
  message_generation
  rospy
)

catkin_python_setup()

add_service_files(
  FILES
  Offline.srv
)

generate_messages(
  DEPENDENCIES
  appctl
  lg_common
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_common appctl message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/lg_offliner_starter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_lg_offliner_basic.test)
  catkin_add_nosetests(test/offline/test_lg_offliner_basic.py)
endif()
