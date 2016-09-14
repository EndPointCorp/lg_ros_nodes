cmake_minimum_required(VERSION 2.8.3)
project(lg_spacenav_globe)

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

add_executable(portal_nav src/portal_nav.cpp)

add_dependencies(portal_nav
  lg_spacenav_globe_generate_messages_cpp
  joystick_navigator
)
add_dependencies(joystick_navigator
  lg_spacenav_globe_generate_messages_cpp
)

target_link_libraries(portal_nav
  ${catkin_LIBRARIES}
  joystick_navigator
)

install(TARGETS portal_nav joystick_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

