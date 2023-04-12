cmake_minimum_required(VERSION 2.8.3)
project(lg_stats)

find_package(catkin REQUIRED COMPONENTS
  appctl
  lg_common
  lg_msg_defs
  rospy
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_common appctl lg_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/stats.py
  scripts/director_stats.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_lg_stats_influx_mock_strategy_default.test)
  catkin_add_nosetests(test/offline/test_lg_stats_basic.py)
endif()
