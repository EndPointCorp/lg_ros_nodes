cmake_minimum_required(VERSION 2.8.3)
project(lg_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ProcessState.msg
  WindowGeometry.msg
)
generate_messages()

catkin_package()

