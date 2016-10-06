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

add_executable(lg_spacenav_globe src/lg_spacenav_globe.cpp)

add_dependencies(lg_spacenav_globe
  lg_spacenav_globe_generate_messages_cpp
  joystick_navigator
)
add_dependencies(joystick_navigator
  lg_spacenav_globe_generate_messages_cpp
)

target_link_libraries(lg_spacenav_globe
  ${catkin_LIBRARIES}
  joystick_navigator
)

install(TARGETS lg_spacenav_globe joystick_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

