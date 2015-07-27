cmake_minimum_required(VERSION 2.8.3)
project(lg_pano)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/master
  scripts/master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# XXX TODO: Come up with tests
# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   catkin_add_nosetests(test/test_query_writer.py)
#   add_rostest(test/kmlsync/kmlsync.test)
# endif()
