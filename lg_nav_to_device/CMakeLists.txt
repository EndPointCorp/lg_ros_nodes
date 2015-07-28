cmake_minimum_required(VERSION 2.8.3)
project(lg_nav_to_device)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/device_writer
  scripts/device_writer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# if (CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test/test_something.py)
# endif()
