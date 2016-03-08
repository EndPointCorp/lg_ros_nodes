cmake_minimum_required(VERSION 2.8.3)
project(spacenav_wrapper)

find_package(catkin REQUIRED
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/spacenav_wrap.py
  scripts/spacenav_wrap
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_spacenav_wrapper.test)
endif()

