cmake_minimum_required(VERSION 2.8.3)
project(spacenav_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

add_library(hid src/hid.c)

add_executable(spacenav_rezero src/spacenav-rezero.cpp)

add_dependencies(spacenav_rezero hid)

target_link_libraries(hid
  ${catkin_LIBRARIES}
  udev
  pthread
  hid
)

target_link_libraries(spacenav_rezero
  ${catkin_LIBRARIES}
  udev
  pthread
  hid
)
install(TARGETS spacenav_rezero hid
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/spacenav_wrap.py
  scripts/spacenav_wrap
  scripts/spacenav_antispin.py
  scripts/spacenav_antispin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/test_spacenav_wrapper.test)
  add_rostest(test/test_spacenav_rezeroer.test)
endif()
