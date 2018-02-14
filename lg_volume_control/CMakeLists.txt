cmake_minimum_required(VERSION 2.8.3)
project(lg_volume_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Volume.srv
)

generate_messages(
  DEPENDENCIES
)


catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

catkin_install_python(PROGRAMS
  scripts/volume_control_master.py
  scripts/volume_control_slave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_volume_control.py)
endif()
