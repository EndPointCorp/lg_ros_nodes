cmake_minimum_required(VERSION 2.8.3)
project(lg_cms_director)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/director.py
  scripts/director
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
