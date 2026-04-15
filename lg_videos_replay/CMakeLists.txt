cmake_minimum_required(VERSION 3.0.2)
project(lg_videos_replay)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  interactivespaces_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lg_videos_play.py
  scripts/videos_replay_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
