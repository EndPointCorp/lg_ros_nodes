cmake_minimum_required(VERSION 2.8.3)
project(lg_activity)

find_package(catkin REQUIRED COMPONENTS
    lg_common
    message_generation
    rospy
    )

catkin_python_setup()

add_message_files(
  FILES
  ActivityState.msg
  )

add_service_files(
  FILES
  ActivityStates.srv
 )

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
  scripts/tracker
  scripts/tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/online/test_tracker.test)
endif()
