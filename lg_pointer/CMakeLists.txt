cmake_minimum_required(VERSION 2.8.3)
project(lg_pointer)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  wiimote
  lg_common
  std_srvs
  geometry_msgs
  sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/wiimote_to_twist.py
  scripts/wiimote_to_pointer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
