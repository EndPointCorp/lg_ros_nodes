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
  ImageViews.msg
  ImageView.msg
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

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/_always_build
  COMMAND ./build_gst_video_sync.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
)
add_custom_target(
  gst_video_sync
  ALL
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/_always_build
)
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/src/gst_video_sync
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

catkin_install_python(PROGRAMS
  scripts/gstreamer
  scripts/gstreamer.py
  scripts/mplayer
  scripts/mplayer.py
  scripts/browser_launcher
  scripts/browser_launcher.py
  scripts/browser_player
  scripts/browser_player.py
  scripts/image_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_mplayer_adhoc_media.test)
  add_rostest(test/online/test_mplayer_director_bridge.test)
  # exclude this test. it's starting real mplayer in a ManagedWindow
  # and it sometimes hangs, exceeds ROS allotted time for a test and fails
  # details: https://github.com/EndPointCorp/lg_ros_nodes/issues/215
  # add_rostest(test/online/test_mplayer_scenarios_real.test)
endif()
