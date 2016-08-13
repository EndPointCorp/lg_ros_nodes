cmake_minimum_required(VERSION 2.8.3)
project(lg_mirror)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS appctl lg_common rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/capture_viewport.py
  scripts/playback.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_aspect_scale.py)
  catkin_add_nosetests(test/test_playback.py)
  add_rostest(test/capture_viewport.test)
  add_rostest(test/viewport_multicast.test)
endif()
