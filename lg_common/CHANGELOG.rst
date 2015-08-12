^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2015-08-12)
------------------
* Fixed state assert
* Change ApplicationState to string field
  This is more human-friendly.
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.0.6 (2015-08-10)
------------------
* ManagedAdhocBrowser
  - enabled online tests for director bridge
  - added some gitignore lines
* ManagedAdhocBrowser
  - added some tests coverage
  - added tests to cmakelists.txt
* AdhocBrowserDirectorBridge tests
  - added basic unit tests
  - updated cmakelist to honor them
* AdhocBrowser
  - added more docstrings
  - added files for rests
  - introduced nosy.cfg for clever nosetesting
* AdhocBrowser
  - added honoring of the offset
* Contributors: Wojciech Ziniewicz

1.0.5 (2015-08-03)
------------------

1.0.4 (2015-07-31)
------------------
* Workaround for Chrome kiosk fullscreen behavior
  Set fullscreen to 'true' in rule properties, then back to false in the callback.
  This seems to be compatible with other applications too.
* Final fixing of managed adhoc browser logic
  - turned loginfo to logdebug here and there
  - fixed helpers
  - general fixing
* Added initial director bridge and dev deployment script
* lg_common helper for extracting asset types for viewports
* AdhocBrowserPool
  - added adhoc browser director bridge
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.0.3 (2015-07-29)
------------------

1.0.2 (2015-07-29)
------------------
* remove redefinition of touchscreen
* Fixed adhoc browser URL handling
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.0.1 (2015-07-29)
------------------

* Removed ambiguous var def thanks to @zdenekmaxa
* Contributors: Wojciech Ziniewicz

0.0.7 (2015-07-28)
------------------
* Show links on center sv
* Contributors: Will Plaut

0.0.6 (2015-07-28)
------------------
* adhoc browser
  - moved everything to lg_common
  - added README for adhoc browser
* ManagedBrowser fix
  - added shutil.rmtree before initialization for --user-data-dir
* Contributors: Will Plaut, Wojciech Ziniewicz

0.0.5 (2015-07-27)
------------------
* Added a helper method to generate url with GET params
* Launch spacenav_node in dev.launch
* Broader search for awesome pid
* Set windows to non-fullscreen, non-maximized
* Eliminate caching in dev_webserver.py
* Contributors: Kannan Ponnusamy, Matt Vollrath, Will Plaut

0.0.4 (2015-07-27)
------------------
* Chamber of understanding
* Contributors: Neil Elliott

0.0.3 (2015-07-21)
------------------
* Fix awesome rule generation errors
* Contributors: Matt Vollrath

0.0.2 (2015-07-21)
------------------
* Allow missing window geometry
* Fix rospy.logerr method names
* Clean up and rename some window mgmt items
* use spawn hook rather than respawn hook
* Added geometry, updated dev.launch with TS
* manage_application: set respawn hook
* Fix imports in awesome script
* Remove xdotool dep
* Use awesome rules for window management
* Revise window searching for reliability over speed
* Add app argument to chrome
* Make the adhoc window showing
* Contributors: Jacob Minshall, Matt Vollrath, Neil Elliott, Szymon Guz, Wojciech Ziniewicz

0.0.1 (2015-07-08)
------------------
* Initial release
* Contributors: Jacob Minshall, Kannan Ponnusamy, Matt Vollrath, Wojciech Ziniewicz
