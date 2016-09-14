cmake_minimum_required(VERSION 2.8.3)
project(lg_mirror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  FILES
  EvdevEvent.msg
  EvdevEvents.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  EvdevDeviceInfo.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS
  appctl
  lg_common
  rospy
  roscpp
  message_runtime
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(uinput_device src/uinput_device.cpp)
add_library(device_service src/device_service.cpp)
add_library(viewport_mapper src/viewport_mapper.cpp)
add_library(ros_event_relay src/ros_event_relay.cpp)
add_library(util src/util.cpp)

add_executable(touch_sender_node src/sender.cpp)
add_executable(touch_receiver_node src/receiver.cpp)

add_dependencies(touch_sender_node
  ${PROJECT_NAME}_generate_messages_cpp
  device_service
)
add_dependencies(touch_receiver_node
  ${PROJECT_NAME}_generate_messages_cpp
  uinput_device
  viewport_mapper
  ros_event_relay
)
add_dependencies(uinput_device
  ${PROJECT_NAME}_generate_messages_cpp
)
add_dependencies(device_service
  ${PROJECT_NAME}_generate_messages_cpp
)
add_dependencies(viewport_mapper
  ${PROJECT_NAME}_generate_messages_cpp
  lg_common_generate_messages_cpp
  util
)
add_dependencies(ros_event_relay
  lg_common_generate_messages_cpp
  uinput_device
  viewport_mapper
)

target_link_libraries(viewport_mapper
  ${catkin_LIBRARIES}
  util
)
target_link_libraries(ros_event_relay
  ${catkin_LIBRARIES}
  uinput_device
  viewport_mapper
)
target_link_libraries(touch_sender_node
  ${catkin_LIBRARIES}
  device_service
)
target_link_libraries(touch_receiver_node
  ${catkin_LIBRARIES}
  uinput_device
  viewport_mapper
  ros_event_relay
)

install(
  TARGETS
  touch_sender_node
  touch_receiver_node
  device_service
  uinput_device
  viewport_mapper
  ros_event_relay
  util
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/capture_viewport_node.py
  scripts/playback_node.py
  scripts/touch_router_node.py
  scripts/capture_viewport_node
  scripts/playback_node
  scripts/touch_router_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_aspect_scale.py)
  catkin_add_nosetests(test/test_playback.py)
  add_rostest(test/capture_viewport.test)
  add_rostest(test/viewport_multicast.test)
  add_rostest(test/touch_router.test)
  add_rostest(test/touch_router_no_default.test)
  catkin_add_gtest(test_${PROJECT_NAME}_viewport_mapper test/test_viewport_mapper.cpp)
  target_link_libraries(test_${PROJECT_NAME}_viewport_mapper
    ${catkin_LIBRARIES}
    viewport_mapper
  )
endif()
