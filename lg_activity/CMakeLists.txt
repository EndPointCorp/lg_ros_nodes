cmake_minimum_required(VERSION 2.8.3)
project(lg_activity)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    lg_common
    rospy
    )

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Activity.srv
 )

generate_messages(
  DEPENDENCIES
  lg_common
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
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

