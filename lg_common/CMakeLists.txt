cmake_minimum_required(VERSION 2.8.3)
project(lg_common)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS message_runtime lg_msg_defs
)

catkin_install_python(PROGRAMS
  scripts/dev_webserver.py
  scripts/static_browser.py
  scripts/adhoc_browser.py
  scripts/state_setter.py
  scripts/touchscreen.py
  scripts/uscs_service.py
  scripts/button.py
  scripts/button
  scripts/dev_webserver
  scripts/static_browser
  scripts/adhoc_browser
  scripts/state_setter
  scripts/touchscreen
  scripts/command_handler_node.py
  scripts/command_handler_node
  scripts/readiness.py
  scripts/readiness
  scripts/uscs_service
  scripts/lg_active_overlay
  scripts/lg_active_overlay.py
  scripts/lg_active_scene
  scripts/lg_active_scene.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY webapps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/offline/test_helpers.py)
  catkin_add_nosetests(test/offline/test_managed_adhoc_browser.py)
  catkin_add_nosetests(test/offline/test_managed_application.py)
  catkin_add_nosetests(test/offline/test_tcp_relay.py)
  catkin_add_nosetests(test/offline/test_uscs_service.py)
  add_rostest(test/online/test_managed_adhoc_browser.test)
  add_rostest(test/online/test_combine_viewport_geometries.test)
endif()
