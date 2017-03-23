cmake_minimum_required(VERSION 2.8.3)
project(lg_sv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_srvs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  PanoIdState.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_srvs
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/uWebSockets/src
  ${catkin_INCLUDE_DIRS}
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/uWebSockets/libuWS.so
  COMMAND make Linux
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/uWebSockets
)

add_custom_target(
  uWS
  ALL
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/uWebSockets/libuWS.so
)
set_target_properties(uWS PROPERTIES COMPILE_FLAGS "--std=c++11")

add_executable(ws_distributor src/ws_distributor.cpp)
add_dependencies(ws_distributor uWS std_srvs_gencpp)
set_target_properties(ws_distributor PROPERTIES COMPILE_FLAGS "--std=c++11")

target_link_libraries(ws_distributor
  ${CMAKE_CURRENT_SOURCE_DIR}/src/uWebSockets/libuWS.so
  ssl
  z
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/launcher.py
  scripts/server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY webapps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/uWebSockets/libuWS.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS
  ws_distributor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/sv_server.test)
  add_rostest(test/sv_panoid_state.test)
endif()
