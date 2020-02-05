cmake_minimum_required(VERSION 2.8.3)
project(lg_earth)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lg_msg_defs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS lg_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/add_kml.py
  scripts/add_kml
  scripts/client
  scripts/client.py
  scripts/query
  scripts/query.py
  scripts/viewsync_relay
  scripts/viewsync_relay.py
  scripts/kmlsync
  scripts/kmlsync.py
  scripts/kmlsync_state
  scripts/kmlsync_state.py
  scripts/planet_changer
  scripts/planet_changer.py
  scripts/toggle_3d_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY webapps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
)

install(
  DIRECTORY bash_scripts/
  DESTINATION /home/lg/bash_scripts
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_query_writer.py)
  catkin_add_nosetests(test/test_query_queue.py)
  add_rostest(test/kmlsync/kmlsync.test)
endif()

