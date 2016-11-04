cmake_minimum_required(VERSION 2.8.3)
project(rfid_scanner)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rfid_scanner
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/reader.py
  scripts/reader
  scripts/sqlite_storage.py
  scripts/sqlite_storage
  scripts/sqlite_uscs_storage.py
  scripts/sqlite_uscs_storage
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
