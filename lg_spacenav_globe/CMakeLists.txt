cmake_minimum_required(VERSION 2.8.3)
project(lg_spacenav_globe)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  lg_msg_defs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp lg_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(joystick_navigator src/joystick_navigator.cpp)

add_executable(lg_spacenav_globe src/lg_spacenav_globe.cpp)

add_dependencies(lg_spacenav_globe
  lg_msg_defs_globe_generate_messages_cpp
  joystick_navigator
)
add_dependencies(joystick_navigator
  lg_msg_defs_generate_messages_cpp
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

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/test_lg_spacenav_globe.test)
endif()
