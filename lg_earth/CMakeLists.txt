cmake_minimum_required(VERSION 2.8.3)
project(lg_earth)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/client
  scripts/client.py
  scripts/query
  scripts/query.py
  scripts/viewsync_relay
  scripts/viewsync_relay.py
  scripts/kmlsync
  scripts/kmlsync.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_query_writer.py)
endif()
