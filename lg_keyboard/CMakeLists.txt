cmake_minimum_required(VERSION 2.8.3)
project(lg_keyboard)

find_package(catkin REQUIRED COMPONENTS
  appctl
  lg_common
  message_generation
  rospy
)

catkin_python_setup()

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

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS
  scripts/onboard
  scripts/onboard.py
  scripts/onboard_router
  scripts/onboard_router.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_onboard_router.test)
  catkin_add_nosetests(test/offline/test_onboard_router.py)
  catkin_add_nosetests(test/offline/test_onboard.py)
endif()
