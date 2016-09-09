cmake_minimum_required(VERSION 2.8.3)
project(spacenav_gmaps)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  message_generation
)

add_message_files(
  FILES
  PortalPose.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(joystick_navigator src/joystick_navigator.cpp)

add_executable(spacenav_gmaps src/portal_nav.cpp)

add_dependencies(spacenav_gmaps
  spacenav_gmaps_generate_messages_cpp
  joystick_navigator
)
add_dependencies(joystick_navigator
  spacenav_gmaps_generate_messages_cpp
)

target_link_libraries(spacenav_gmaps
  ${catkin_LIBRARIES}
  joystick_navigator
)

install(TARGETS portal_nav joystick_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

