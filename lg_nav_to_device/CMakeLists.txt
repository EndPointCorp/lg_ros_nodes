cmake_minimum_required(VERSION 2.8.3)
project(lg_nav_to_device)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# if (CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test/test_something.py)
# endif()
