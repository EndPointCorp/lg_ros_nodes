cmake_minimum_required(VERSION 2.8.3)
project(lg_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ApplicationState.msg
  WindowGeometry.msg
  AdhocBrowser.msg
  AdhocBrowsers.msg
)
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/dev_webserver.py
  scripts/static_browser.py
  scripts/adhoc_browser.py
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
  catkin_add_nosetests(test/offline/test_managed_application.py)
  add_rostest(test/online/test_managed_adhoc_browser_director_bridge.test)
  catkin_add_nosetests(test/offline/test_managed_adhoc_browser.py)
endif()
