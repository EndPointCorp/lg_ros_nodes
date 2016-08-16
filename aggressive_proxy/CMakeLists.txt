cmake_minimum_required(VERSION 2.8.3)
project(aggressive_proxy)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/proxy.py
  scripts/proxy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_proxy_handler.py)
  add_rostest(test/aggressive_proxy/proxy.test)
endif()
