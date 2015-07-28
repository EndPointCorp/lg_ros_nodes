cmake_minimum_required(VERSION 2.8.3)
project(lg_earth)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  KmlState.srv
  PlaytourQuery.srv
  LastSceneModified.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

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
  scripts/kmlsync_state
  scripts/kmlsync_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_query_writer.py)
  add_rostest(test/kmlsync/kmlsync.test)
endif()

