cmake_minimum_required(VERSION 2.8.3)
project(lg_attract_loop)

find_package(catkin REQUIRED COMPONENTS
    lg_common
    message_generation
    rospy
    )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  lg_common
  )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_common message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/attract_loop
  scripts/attract_loop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
