cmake_minimum_required(VERSION 2.8.3)
project(rosparam_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/param_server.py
  scripts/param_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
