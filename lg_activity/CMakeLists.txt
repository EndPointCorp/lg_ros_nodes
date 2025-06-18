cmake_minimum_required(VERSION 2.8.3)
project(lg_activity)

find_package(catkin REQUIRED COMPONENTS
    lg_msg_defs
    rospy
    )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs lg_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/dcv_activity_pub.py
  scripts/dcv_activity_pub
  scripts/write_scene_timestamps.py
  scripts/write_scene_timestamps
  scripts/tracker.py
  scripts/tracker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  #add_rostest(test/online/test_tracker.test)
endif()
