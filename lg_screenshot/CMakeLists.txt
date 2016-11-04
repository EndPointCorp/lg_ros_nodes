cmake_minimum_required(VERSION 2.8.3)
project(lg_screenshot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  GetScreenshot.msg
  Screenshot.msg
)
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/web_screenshot.py
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
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/offline/test_helpers.py)
  catkin_add_nosetests(test/offline/test_managed_adhoc_browser.py)
  catkin_add_nosetests(test/offline/test_managed_application.py)
  catkin_add_nosetests(test/offline/test_tcp_relay.py)
  catkin_add_nosetests(test/offline/test_uscs_service.py)
  add_rostest(test/online/test_managed_adhoc_browser_director_bridge.test)
  add_rostest(test/online/test_managed_adhoc_browser.test)
  add_rostest(test/online/state_changer.test)
  add_rostest(test/online/uscs_service.test)
  add_rostest(test/online/uscs_initial_state_adhoc_browser.test)
  add_rostest(test/online/test_adhoc_browser_pool_rosbridge_params.test)
  add_rostest(test/online/test_adhoc_browser_url_monitor.test)
endif()
