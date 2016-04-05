^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.40 (2016-03-23)
-------------------

1.1.39 (2016-03-16)
-------------------

1.1.38 (2016-03-09)
-------------------

1.1.37 (2016-03-04)
-------------------
* managed_browser: extensions loading
* Contributors: Jacob Minshall

1.1.36 (2016-02-17)
-------------------
* lg\_{common,earth}: set initial state
  This will allow us to start up a hidden window initially.
* Contributors: Jacob Minshall

1.1.35 (2016-02-05)
-------------------

1.1.34 (2016-02-05)
-------------------

1.1.33 (2016-02-04)
-------------------

1.1.32 (2016-01-28)
-------------------

1.1.31 (2016-01-20)
-------------------
* static_browser: fix issues with undefined variables
* Contributors: Jacob Minshall

1.1.30 (2016-01-11)
-------------------
* pdfviewer: stretches to either height and width of screen
  This ends up only stretching to the width, which is fine because of the
  ratio. Now some good geometry will be needed to make things look pretty.
* Contributors: Jacob Minshall

1.1.29 (2016-01-04)
-------------------

1.1.28 (2015-12-10)
-------------------
* command_handler: added code to command and listener node
* Contributors: Jacob Minshall

1.1.27 (2015-11-25)
-------------------

1.1.26 (2015-11-25)
-------------------

1.1.25 (2015-11-17)
-------------------

1.1.24 (2015-11-16)
-------------------

1.1.23 (2015-11-13)
-------------------

1.1.22 (2015-11-05)
-------------------

1.1.21 (2015-10-22)
-------------------

1.1.20 (2015-10-21)
-------------------
* Fix Chrome 46 window management
* Contributors: Matt Vollrath

1.1.19 (2015-10-20)
-------------------

1.1.18 (2015-10-20)
-------------------

1.1.17 (2015-10-16)
-------------------
* touchscreen: removed state from touchscreen
  There didn't seem to be a reason to hide the touchscreen, and this
  makes the state changer a pain to interact with. Changes will come
  to the state changer in the future to allow ignoring certain REs
  matching topics.
* Contributors: Jacob Minshall

1.1.16 (2015-10-11)
-------------------

1.1.15 (2015-10-10)
-------------------

1.1.14 (2015-10-08)
-------------------

1.1.13 (2015-10-08)
-------------------
* Better handling of timeouts
* Contributors: Adam Vollrath

1.1.12 (2015-10-07)
-------------------
* state_changer: test: sleep before publishing
  The array based mock state uncovered an issue, the very first time these
  publishers were created, they didn't actually publish anything, even
  with the wait_for_pub in there.
* state_changer: test: use array of states to check for extra messages
* state_changer: add rostest to cmake
* pep8: ignore E265, block comments requiring space after #
* state_changer: test
* state_changer: sleep for a second when creating new publishers
  Creating a publisher and then publishing on it right after seems to not
  work very well in rospy. This mitigates that problem.
* state_changer: StringArray is an array of actual strings, not Strings
  There is no string.data, msg.strings is an actual array of strings,
  which python can handle just fine.
* state_changer: moved class to it's own file for testing
* state_changer: use an array of strings
  Multiple activities can be passed to the state changer, and only those
  ones will be set to VISIBLE.
* lg_common: added the StringArray type
* link to state_changer.py
* lg_common: added a state handler/changer
  This will publish HIDDEN to all other state listening topics, except for
  the one specified in the string passed to /state_handler/activate
* Contributors: Jacob Minshall, Wojciech Ziniewicz

1.1.11 (2015-10-06)
-------------------
* Fixed touchscreen typo
* Contributors: Wojciech Ziniewicz

1.1.10 (2015-10-05)
-------------------
* Added lots of docs
* Fixed pep8

1.1.9 (2015-09-25)
------------------
* Dont start application if X is not available
* Better logging for dependencies
* Added missing imports
* ADded dependency checking and fixed slots deserialization
* lg_replay: retain permissions on other event devices
* lg_replay: lg_common: make sure we iterate over tuple
  If __slots_\_ only has one value, it returns a string. Now we turn that
  string into a tuple instead of trying to iterate over each character in
  the string.
* Make kmlsync work better by default, kill Futurama
* pep8 fixes
* Added ext dependency mechanism and added it to GE and SV/PV
* Some debug for TS
* Contributors: Adam Vollrath, Jacob Minshall, Joshua Tolley, Wojciech Ziniewicz

1.1.8 (2015-09-25)
------------------

1.1.7 (2015-09-24)
------------------

1.1.6 (2015-09-24)
------------------

1.1.5 (2015-09-23)
------------------

1.1.4 (2015-09-23)
------------------

1.1.3 (2015-09-22)
------------------

1.1.2 (2015-09-22)
------------------

1.1.1 (2015-09-18)
------------------

1.1.0 (2015-09-17)
------------------
* Fixed the path for the TS
* Added touchscreen launcher
* lg\_{common,sv}: used the new director listener abstraction
* lg_common: abstract director message subscribing
* lg_common: throw exception when loading a director message fails
* Added shell to managed browser to prevent pid leakage
* lg_common: added more director helpers inside
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.0.9 (2015-09-09)
------------------

1.0.8 (2015-08-12)
------------------

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
