cmake_minimum_required(VERSION 2.8.3)
project(lg_media)

find_package(catkin REQUIRED COMPONENTS
  lg_common
  message_generation
  rospy
)

catkin_python_setup()

add_message_files(
  FILES
  AdhocMedia.msg
  AdhocMedias.msg
)

add_service_files(
  FILES
  MediaAppsInfo.srv
)

generate_messages(
  DEPENDENCIES
  lg_common
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_common message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/mplayer
  scripts/mplayer.py
  scripts/browser_launcher
  scripts/browser_launcher.py
  scripts/browser_player
  scripts/browser_player.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_mplayer_adhoc_media.test)
  add_rostest(test/online/test_mplayer_director_bridge.test)
  add_rostest(test/online/test_mplayer_scenarios_real.test)
endif()