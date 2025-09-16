cmake_minimum_required(VERSION 2.8.3)
project(lg_ollama)

find_package(catkin REQUIRED COMPONENTS
    visionport
    )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS visionport std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/stt_bridge
  scripts/stt_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

