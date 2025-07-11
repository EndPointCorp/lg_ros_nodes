^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2020-05-27)
------------------
* Move bspc rule commands further down the stack
* Manage bspc rules from ManagedWindow
  lg_wm now just converges on its internal window list.
* Use lg_wm_send instead of awesome-client

3.5.0 (2022-03-15)
------------------

3.4.1 (2022-02-03)
------------------

3.4.0 (2022-01-12)
------------------
* Merge branch 'master' of github.com:endpointcorp/lg_ros_nodes into fake_spacenav
* Contributors: Jacob Minshall

3.3.16 (2021-12-16)
-------------------
* Merge branch 'touchscreen_toggle_bool' of github.com:endpointcorp/lg_ros_nodes
* now uses a bool and can use any viewport / url
* Contributors: Jacob Minshall

3.3.15 (2021-12-09)
-------------------

3.3.14 (2021-11-23)
-------------------
* remove debugging
* use touchscreen_button viewport
* wait longer, maybe fix test
* link
* removing actual_button for browser based and adding to cmake
* button is now a chrome app
* make the linter happy
* remove debugging
* fix relaunch for button
* actual button and ros node added
* moved button to own file and set touch layer
* Contributors: Jacob Minshall

3.3.13 (2021-09-21)
-------------------

3.9.1 (2024-12-17)
------------------

3.9.3 (2025-04-02)
------------------
* Remove crufty rospy import
* Write sys.deps for all packages
* Contributors: Matt Vollrath

3.9.5 (2025-04-08)
------------------

3.9.4 (2025-04-08)
------------------
* 3.9.3
* Changelogs
* Remove crufty rospy import
* Write sys.deps for all packages
* Contributors: Matt Vollrath

3.9.25 (2025-06-26)
-------------------
* Use VPROS_LOG_LEVEL env
* Log to stderr
* Cleanup logging
  Change messages and adjust log levels.
* Contributors: Matt Vollrath

3.9.24 (2025-06-26)
-------------------

3.9.23 (2025-06-16)
-------------------
* clean up logs and make the data dir before anything else
* Contributors: Jacob Minshall

3.9.22 (2025-06-12)
-------------------
* file locking apt install python3-filelock
* WIP getting things working on ephq
* Contributors: Galaxy Admin

3.9.21 (2025-06-11)
-------------------
* Don't raise when field can't be pickled
* Fix misldeading message_is_nonzero docstring
* Contributors: Matt Vollrath

3.9.20 (2025-06-09)
-------------------
* multiple browsers using the same user data dir
  Copy the user data dir over if it is already in use
* Contributors: Jacob Minshall

3.9.19 (2025-05-20)
-------------------

3.9.18 (2025-05-19)
-------------------

3.9.17 (2025-05-15)
-------------------
* Wait for initial state service before calling
* Contributors: Matt Vollrath

3.9.15 (2025-04-25)
-------------------
* extra global param to remove default args
* Contributors: Jacob Minshall

3.9.14 (2025-04-22)
-------------------

3.9.13 (2025-04-21)
-------------------

3.9.12 (2025-04-15)
-------------------

3.9.11 (2025-04-15)
-------------------

3.9.10 (2025-04-15)
-------------------

3.9.9 (2025-04-15)
------------------

3.9.8 (2025-04-14)
------------------
* Note about get_package_path error case
* Use get_package_path
  Cleanup some unused rospkg imports.
* Add get_package_path helper
  Refactor this pattern for portability.
* Contributors: Matt Vollrath

3.9.7 (2025-04-11)
------------------
* Clean up legacy roslib.load_manifest() call
* Contributors: Matt Vollrath

3.9.6 (2025-04-09)
------------------
* fixing up user data dir
  also adds a script to sync onto running docker
* 3.9.5
* bump
* 3.9.4
* bump
* 3.9.3
* Changelogs
* Remove crufty rospy import
* Write sys.deps for all packages
* Contributors: Jacob Minshall, Matt Vollrath

3.9.2 (2024-12-23)
------------------
* fix for sv to grab data out of asset instead of activity config
* 3.9.1
* Changelogs
* Contributors: Jacob Minshall, Matt Vollrath

3.9.0 (2024-02-06)
------------------

3.8.5 (2024-01-09)
------------------

3.8.4 (2023-12-29)
------------------

3.8.3 (2023-10-17)
------------------

3.8.2 (2023-10-06)
------------------
* Remove duplicated flags from managed browser
* Contributors: Alejandro Ramon

3.8.1 (2023-10-06)
------------------

3.8.0 (2023-05-25)
------------------
* Merge pull request `#416 <https://github.com/endpointcorp/lg_ros_nodes/issues/416>`_ from EndPointCorp/quiet_logs_some
  log quieting
* big logging changes
* Merge branch 'master' of github.com:endpointcorp/lg_ros_nodes into quiet_logs_some
* using each browsers tmpdir
* log quieting
* Contributors: Jacob Minshall

3.7.2 (2023-04-27)
------------------
* no more static robotwebtools
* Contributors: Jacob Minshall

3.7.1 (2023-04-06)
------------------
* small fixes to get static browser working properly
* Contributors: Jacob Minshall

3.7.0 (2023-03-30)
------------------

3.6.0 (2022-11-22)
------------------
* add png image for touchscreen toggle button
* Remove chrome options that are causing issues with scene performance
* Contributors: Alejandro Ramon, Constante "Tino" Gonzalez

* add png image for touchscreen toggle button
* Remove chrome options that are causing issues with scene performance
* Contributors: Alejandro Ramon, Constante "Tino" Gonzalez

3.5.5 (2022-09-09)
------------------
* scalable png, grey background
* Contributors: Constante "Tino" Gonzalez

3.5.4 (2022-08-10)
------------------
* Remove rawdraw feature from managed browsers, add other additional flags for performance
* Contributors: Alejandro Ramon

3.5.3 (2022-05-27)
------------------
* Update default managed_browser flags to support newer browsers
* Contributors: Galaxy Admin

3.5.2 (2022-03-29)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Galaxy Admin

3.5.1 (2022-03-24)
------------------
* Merge pull request `#440 <https://github.com/endpointcorp/lg_ros_nodes/issues/440>`_ from EndPointCorp/nav_mode_earth_state
  Nav mode earth state
* untested unknown how this will work!
* 3.5.0
* bump changelogs
* 3.4.1
* Changelogs
* 3.4.0
* bump changelogs
* Merge branch 'master' of github.com:endpointcorp/lg_ros_nodes into fake_spacenav
* 3.3.16
* bump changelogs
* Merge branch 'touchscreen_toggle_bool' of github.com:endpointcorp/lg_ros_nodes
* 3.3.15
* bump changelogs
* now uses a bool and can use any viewport / url
* 3.3.14
* changelogs
* remove debugging
* use touchscreen_button viewport
* wait longer, maybe fix test
* link
* removing actual_button for browser based and adding to cmake
* button is now a chrome app
* make the linter happy
* remove debugging
* fix relaunch for button
* actual button and ros node added
* moved button to own file and set touch layer
* 3.3.13
* bump changelogs
* Contributors: Jacob Minshall, Matt Vollrath, Will Plaut

3.3.12 (2021-08-10)
-------------------

3.3.11 (2021-07-23)
-------------------

3.3.10 (2021-07-22)
-------------------
* Make ad hoc browser LAYER_ABOVE the exception
* Default to LAYER_NORMAL
* Ad hoc browsers at LAYER_ABOVE
* Add layer constants to ManagedWindow
* Contributors: Matt Vollrath

3.3.9 (2021-07-20)
------------------
* Revert "Revert "Revert "Revert "Use lg_wm_send instead of awesome-client""""
  This reverts commit d134f8bb67f72f7c21e8c9b04864af6e3caddeda.
* Contributors: Jacob Minshall

3.3.8 (2021-01-29)
------------------

3.3.7 (2021-01-14)
------------------
* fix xdo tools browser search string
* Contributors: dkiselev

3.3.6 (2021-01-06)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* reload Cesium wall brawsers
* Contributors: dkiselev

3.3.5 (2020-12-28)
------------------

3.3.4 (2020-12-11)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* fix typo
* Contributors: dkiselev

3.3.3 (2020-12-10)
------------------
* add Chrome class to xdotools search
* pep8 it
* Contributors: Will Plaut, dkiselev

3.3.2 (2020-12-09)
------------------
* reload chrome browser on aw snap with F5
* reload page on aw snap
* relaunch browser on aw snap
* aw snaps reloading
* Contributors: dkiselev

3.3.1 (2020-10-29)
------------------

3.3.0 (2020-10-21)
------------------

3.2.9 (2020-09-21)
------------------

3.2.8 (2020-08-27)
------------------

3.2.7 (2020-08-25)
------------------
* Remove BSP  (`#427 <https://github.com/endpointcorp/lg_ros_nodes/issues/427>`_)
  * Don't lg_wm_send instead of awesome-client
  * Don't Put Earth windows below
  * Manage bspc rules from ManagedWindow
  * Move bspc rule commands further down the stack
* Contributors: nelliott

3.2.6 (2020-07-09)
------------------
* Revert "Revert "Move bspc rule commands further down the stack""
  This reverts commit 18a6966bf98c1c06a548273f64a5f4d189eeb841.
* Revert "Revert "Manage bspc rules from ManagedWindow""
  This reverts commit bdcf3b6b040a654c43dcbb2e8114270a3c305a56.
* Revert "Revert "Use lg_wm_send instead of awesome-client""
  This reverts commit 75c9e05388298c614927bc5552a79c60ed4d8089.
* Contributors: Neil Elliott

* Revert "Revert "Move bspc rule commands further down the stack""
  This reverts commit 18a6966bf98c1c06a548273f64a5f4d189eeb841.
* Revert "Revert "Manage bspc rules from ManagedWindow""
  This reverts commit bdcf3b6b040a654c43dcbb2e8114270a3c305a56.
* Revert "Revert "Use lg_wm_send instead of awesome-client""
  This reverts commit 75c9e05388298c614927bc5552a79c60ed4d8089.
* Contributors: Neil Elliott

3.2.5 (2020-07-06)
------------------

3.2.4 (2020-05-29)
------------------

3.2.3 (2020-05-28)
------------------
* Revert "Use lg_wm_send instead of awesome-client"
  This reverts commit 23c87310b80df0b502ffdd1ca72f079bc43f220c.
* Revert "Manage bspc rules from ManagedWindow"
  This reverts commit 5cd056a3c7c06b314b12a84bc47abd5eee37984d.
* Revert "Move bspc rule commands further down the stack"
  This reverts commit 42d9c305bb0a0084b56d461c7047c45744d71029.
* Contributors: Galaxy Admin, Neil Elliott

3.2.2 (2020-05-28)
------------------

3.2.1 (2020-05-27)
------------------
* 3.2.0
* BSPWM
* Move bspc rule commands further down the stack
* Manage bspc rules from ManagedWindow
  lg_wm now just converges on its internal window list.
* Use lg_wm_send instead of awesome-client
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Galaxy Admin, Matt Vollrath, Neil Elliott, dkiselev

3.1.12 (2020-05-04)
-------------------

3.1.11 (2020-05-01)
-------------------
* Skip some unreliable tests
* Merge pull request `#419 <https://github.com/EndPointCorp/lg_ros_nodes/issues/419>`_ from EndPointCorp/lg_common_relative_topics
  Use relative topic names in adhoc browser
* Use relative topic names in adhoc browser
  Allow namespaced duplicates.
* Update Jenkins usage and Fix tests (`#417 <https://github.com/EndPointCorp/lg_ros_nodes/issues/417>`_)
  * Setup step
  * run_ros_setup
  * add more setup tasks
  * syntax changes
  * foo
  * Added Jenkinsfile
  * blah
  * foo
  * blahhhh
  * gahhh
  * bar
  * setup_tests and jenkinsfile stuff
  * Jenkinsfile: fix syntax
  * Jenkinsfile: fix syntax2
  * Jenkinsfile: fix syntax3
  * jenkinsfile stuff
  * Fix perms
  * setup_tests: more changes
  * setup_tests
  * more setup
  * fix stufff
  * remove weird setup stage
  * fix stuff
  * fix stuff
  * Jenkinsfile
  * Jenkinsfile
  * Jenkinsfile: use diff syntax
  * More changes
  * remove auth sock stuff for now
  * Jenkins file tweaks
  * Wrong env for vars
  * blah
  * Jenkinsfile pause on input
  * more jenkinsfile fixin
  * add debugging pause
  * Install pepperflash
  * Dockerfile syntax fixes
  * Dockerfile changes
  * pepperflash
  * Appctl changes in our tests
  * Dockerfile cleanup
  * Fix remaining stats issue
  * Try mounting pepperflash
  * Fix mistakes
  * Try increasing timeout
  * Don't install pepperflash
  * Ban persistent ServiceProxys
  * Try increasing load time
  * Install pycryptodome
  * Remove unused import from test
  * Try installing hacked rosbridge
  * Fix spelling
  * Fix unit test and try longer grace period
  * Fix test (dont use bytearray)
  * try again
  * Cleanup rosbridge install
  * Fix syntax
  * Change listener.js and add rosbridge
  * fix listener issue
  * revert most listener changes
  * more changes to listener
  * wait_for_assert fix
  THANKS JACOB
  * fix test_helper
  * Add time to two remaining fails
  * more changes to listener
  * more changes to listener
  * Use new assert gt
  Thanks jacob
  * import updated helper
  * Greater than or equal
  * fix stuff
  * ftw!
  * pep8 changes
  * Combine lg_common tests
  * Fixup lg_screenshot and lg_keyboard tests
  * fix import
  * pycodestyle!
  * Don't --rm
  * Cleanup Jenkinsfile
  * Add step for master builds
  * Set env when running tests
  * Test commit
  Co-authored-by: Jacob Minshall <jacob@endpoint.com>
* Contributors: Matt Vollrath, Will Plaut

3.1.10 (2020-03-25)
-------------------

3.1.9 (2020-03-11)
------------------

3.1.8 (2020-02-06)
------------------

3.1.7 (2020-02-04)
------------------
* Set timeout when writing to influx
  Reduces shutdown delay when the telegraf server is unreachable.
* Remove post-exception handling sleep
  Blocking socket should not need this.
* Contributors: Matt Vollrath

3.1.6 (2020-01-27)
------------------

3.1.5 (2020-01-24)
------------------

3.1.4 (2020-01-24)
------------------
* Merge pull request `#414 <https://github.com/EndPointCorp/lg_ros_nodes/issues/414>`_ from EndPointCorp/topic/image_viewer_fixes_and_tracebacks
  Topic/image viewer fixes and tracebacks
* changes need to be cleaned up
* Contributors: Will Plaut

3.1.3 (2020-01-20)
------------------
* Merge pull request `#413 <https://github.com/EndPointCorp/lg_ros_nodes/issues/413>`_ from EndPointCorp/topic/fix_js_msg_types
  lots of missing msg/srv updates in js files
* random problems that need fixin
* lots of missing msg/srv updates in js files
* Merge pull request `#412 <https://github.com/EndPointCorp/lg_ros_nodes/issues/412>`_ from EndPointCorp/no_waiting
  Fix some service waiting and other issues
* Ban persistent ServiceProxy
  Known to be broken in this configuration.
* Retry initial USCS state
* Don't wait for pubsub connections either
* Don't wait for services
* Contributors: Matt Vollrath, Will Plaut

3.1.2 (2020-01-10)
------------------

3.1.1 (2020-01-08)
------------------

3.1.0 (2020-01-06)
------------------
* Merge pull request `#411 <https://github.com/EndPointCorp/lg_ros_nodes/issues/411>`_ from EndPointCorp/topic/msg_cleanup
  Topic/msg cleanup
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/msg_cleanup
  Conflicts:
  lg_common/package.xml
* Merge pull request `#410 <https://github.com/EndPointCorp/lg_ros_nodes/issues/410>`_ from EndPointCorp/fix_tests
  Fix some tests
* cleanup deps with catkin_lint
* Merge branch 'master' into fix_tests
* Update CMake and package xmls
* Remove msg creation from orig pkgs
* Update service imports
* Fix up adhoc browser tests
* Fix some pycodestyle glitches
* update import paths everywhere
* Track socat dependency again
* Fix TCP Relay test
  Needs to init a rospy node for appctl to work properly.
  Also fix threading issues when the relay fails to run.
* Contributors: Matt Vollrath, Will Plaut

3.0.2 (2019-11-06)
------------------

3.0.1 (2019-11-06)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* lg_common: remove socat dep that breaks stuffff
  Figure out if we can add this back
* lg_common: add build dep
* Contributors: Will Plaut, dkiselev

3.0.0 (2019-10-31)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/python_tree
  Conflicts:
  rosbridge_library/CHANGELOG.rst
  rosbridge_library/package.xml
  rosbridge_server/CHANGELOG.rst
  rosbridge_server/package.xml
* lg_common: python3 changes
* Fix lg_common test_helpers
* set python executable for tests
* Fix byteness in TCP relay test
* 2to3 all of it
* Merge branch 'topic/python_tree' of github.com:EndPointCorp/lg_ros_nodes into topic/python_tree
* sock.send[all/to]: now takes bytes not str
* python 2 shebang to 3
* Contributors: Matt Vollrath, Will Plaut

2.0.18 (2019-10-11)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/image_checker
* Contributors: Will Plaut

2.0.17 (2019-09-11)
-------------------
* Increase Chrome update check interval
  This should give us a few weeks.
* Contributors: Matt Vollrath

2.0.16 (2019-09-06)
-------------------

2.0.15 (2019-08-20)
-------------------

2.0.14 (2019-08-19)
-------------------

2.0.13 (2019-07-29)
-------------------

2.0.12 (2019-07-24)
-------------------

2.0.11 (2019-07-22)
-------------------

2.0.10 (2019-07-18)
-------------------
* Bypass new Chrome autoplay policy
  Appreciated when browsing, annoying for kiosk apps.
  This should fix panovideo master and any other broken media pages.
* Contributors: Matt Vollrath

2.0.9 (2019-07-17)
------------------
* Merge pull request `#403 <https://github.com/EndPointCorp/lg_ros_nodes/issues/403>`_ from EndPointCorp/fix_bionic_kmlsync
  kmlsync: Encode outgoing text
* Fix combine_viewport_geometries test
* Contributors: Matt Vollrath

2.0.8 (2019-07-08)
------------------
* Move combine_viewport_geometries to lg_common
* Contributors: Matt Vollrath

2.0.7 (2019-07-03)
------------------
* Normalize roslib topic throttle and queues
* Contributors: Matt Vollrath

2.0.6 (2019-07-02)
------------------

2.0.5 (2019-07-02)
------------------

2.0.4 (2019-07-02)
------------------

2.0.3 (2019-07-02)
------------------

2.0.2 (2019-07-01)
------------------

2.0.1 (2019-06-28)
------------------

2.0.0 (2019-06-14)
------------------

1.20.4 (2019-06-12)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/kml_alive
* Contributors: Galaxy Admin

1.20.3 (2019-05-22)
-------------------

1.20.2 (2019-05-22)
-------------------
* Revert "Revert "Extra fullscreen signal fix for Chrome kiosk""
  Turns out this wasn't the problem.
* Include width and height in window callback
* Contributors: Matt Vollrath

1.20.1 (2019-05-21)
-------------------
* Revert "Extra fullscreen signal fix for Chrome kiosk"
  This fix turned out to not be backwards compatible.
* Contributors: Matt Vollrath

1.20.0 (2019-05-15)
-------------------
* PEP8 sweep
* Fix jslint errors
* String form for ManagedWindow
  A little treat for debugging.
* Extra fullscreen signal fix for Chrome kiosk
  Had an issue with new awesome where Chrome in kiosk mode would switch
  back to fullscreen after the callback had run.  This should prevent any
  managed client from unwanted fullscreening.
* Remove backslashes from Chrome instance match
  Breaks in new awesome.
* Isolate awesome environment
  Don't muck with the parent process environment.
* Set fullscreen attribute in rule
  Compatibility with new awesome.
* Contributors: Matt Vollrath

1.19.16 (2019-05-14)
--------------------

1.19.15 (2019-04-29)
--------------------

1.19.14 (2019-04-26)
--------------------

1.19.13 (2019-04-25)
--------------------

1.19.12 (2019-03-25)
--------------------
* Fix awesome rule check
* Contributors: Matt Vollrath

1.19.11 (2019-03-20)
--------------------
* Fix awesome copmatibility
  In newer versions of awesome, rules may not have a 'rule' field.
  We know ours do, so check for rules field as a pre-condition.
* Contributors: Matt Vollrath

1.19.10 (2019-03-15)
--------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Dmitry Kiselev

1.19.9 (2019-03-06)
-------------------

1.19.8 (2019-02-26)
-------------------

1.19.7 (2019-02-14)
-------------------

1.19.6 (2019-02-08)
-------------------

1.19.5 (2019-02-06)
-------------------
* add empty response return for service call
* fix message republishing
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Add /uscs/republish service
* Contributors: Dmitry Kiselev

1.19.4 (2019-01-30)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Dmitry Kiselev

1.19.3 (2019-01-29)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Dmitry Kiselev

1.19.2 (2019-01-11)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Dmitry Kiselev

1.19.1 (2019-01-11)
-------------------

1.19.0 (2019-01-10)
-------------------

1.18.22 (2018-12-06)
--------------------

1.18.21 (2018-12-05)
--------------------
* Merge pull request `#392 <https://github.com/EndPointCorp/lg_ros_nodes/issues/392>`_ from EndPointCorp/topic/user_data_dirrrr
  Topic/user data dirrrr
* user_data_dir: working so far
* initial commit for user_data_dir stuffz
* Contributors: Will Plaut

1.18.20 (2018-11-28)
--------------------

1.18.19 (2018-10-26)
--------------------

1.18.18 (2018-10-12)
--------------------

1.18.17 (2018-10-01)
--------------------

1.18.16 (2018-09-12)
--------------------

1.18.15 (2018-08-24)
--------------------

1.18.14 (2018-07-18)
--------------------

1.18.13 (2018-06-22)
--------------------

1.18.12 (2018-06-05)
--------------------

1.18.11 (2018-05-22)
--------------------

1.18.10 (2018-05-17)
--------------------

1.18.9 (2018-05-14)
-------------------

1.18.8 (2018-05-07)
-------------------
* Hide adhoc browser overlay via ros
* Contributors: Dmitry Kiselev

1.18.7 (2018-05-04)
-------------------
* install extension
* Contributors: Dmitry Kiselev

1.18.6 (2018-05-03)
-------------------
* Add close window extension
* Contributors: Dmitry Kiselev

1.18.5 (2018-05-02)
-------------------

1.18.4 (2018-04-04)
-------------------

1.18.3 (2018-04-03)
-------------------

1.18.2 (2018-04-02)
-------------------

1.18.1 (2018-03-09)
-------------------
* Add default flags for chrome to get touch propperly working with TS
* Contributors: Dmitry Kiselev

1.18.0 (2018-02-26)
-------------------

1.17.14 (2018-02-21)
--------------------

1.17.13 (2018-02-16)
--------------------

1.17.12 (2018-01-09)
--------------------

1.17.11 (2017-12-26)
--------------------

1.17.10 (2017-12-26)
--------------------

1.17.9 (2017-12-18)
-------------------

1.17.8 (2017-12-13)
-------------------

1.17.7 (2017-12-12)
-------------------
* removing un-needed flag
* Contributors: Jacob Minshall

1.17.6 (2017-11-15)
-------------------

1.17.5 (2017-11-14)
-------------------

1.17.4 (2017-11-10)
-------------------

1.17.3 (2017-11-07)
-------------------

1.17.2 (2017-11-06)
-------------------
* super security (long live the wojo)
* Contributors: Jacob Minshall

1.17.1 (2017-10-12)
-------------------
* Merge pull request `#376 <https://github.com/endpointcorp/lg_ros_nodes/issues/376>`_ from EndPointCorp/topic/insecure_content
  allow insecure content
* allow insecure content
  This will help us show pages where our extension is using either http
  or https, and the page its on is using the opposite.
* Contributors: Jacob Minshall

1.17.0 (2017-10-06)
-------------------

1.16.1 (2017-08-17)
-------------------
* Fix error upon closing a ManagedApplication
  We never stored the env value.
* Contributors: Matt Vollrath

1.16.0 (2017-08-17)
-------------------
* Add env arg to ManagedApplication
* Contributors: Matt Vollrath

1.15.0 (2017-08-07)
-------------------

1.14.2 (2017-08-02)
-------------------

1.14.1 (2017-07-17)
-------------------

1.14.0 (2017-07-14)
-------------------

1.13.5 (2017-06-29)
-------------------

1.13.4 (2017-06-13)
-------------------
* fix the generated hash (new field means new hash)
* remove un-needed todo
* ability to remove default arguments
* Contributors: Jacob Minshall

1.13.3 (2017-05-31)
-------------------

1.13.2 (2017-05-23)
-------------------

1.13.1 (2017-05-19)
-------------------

1.13.0 (2017-05-19)
-------------------

1.12.5 (2017-05-11)
-------------------

1.12.4 (2017-05-11)
-------------------
* Fix PEP8
* Contributors: Matt Vollrath

1.12.3 (2017-05-03)
-------------------
* Copy PNaCl Chrome component before browser launch. `#357 <https://github.com/EndPointCorp/lg_ros_nodes/issues/357>`_
* Contributors: Adam Vollrath

1.12.2 (2017-04-26)
-------------------

1.12.1 (2017-04-24)
-------------------

1.12.0 (2017-04-20)
-------------------

1.11.4 (2017-04-06)
-------------------

1.11.3 (2017-03-31)
-------------------

1.11.2 (2017-03-31)
-------------------

1.11.1 (2017-03-28)
-------------------

1.11.0 (2017-03-27)
-------------------
* initial sv on director message
  Nearby panos broken when a director scene is published
* Contributors: Jacob Minshall

1.10.2 (2017-03-24)
-------------------

1.10.1 (2017-03-23)
-------------------

1.10.0 (2017-03-23)
-------------------
* Added exception handling during relaunches for lg_replay and lg_sv (`#345 <https://github.com/EndPointCorp/lg_ros_nodes/issues/345>`_)
* Contributors: Wojciech Ziniewicz

1.9.1 (2017-03-20)
------------------

1.9.0 (2017-03-20)
------------------

1.8.0 (2017-03-09)
------------------
* Add kiosk param to static_browser.py
* Add static_browser.py to README
* Contributors: Matt Vollrath

1.7.11 (2017-03-03)
-------------------

1.7.10 (2017-03-02)
-------------------

1.7.9 (2017-03-01)
------------------
* using links to the flash directory
* Contributors: Jacob Minshall

1.7.8 (2017-03-01)
------------------

1.7.7 (2017-02-28)
------------------
* flash is now copied into chrome user data dirs
* Contributors: Jacob Minshall

1.7.6 (2017-02-27)
------------------

1.7.5 (2017-02-27)
------------------
* Fix missing ApplicationState in lg_common helpers
* Contributors: Matt Vollrath

1.7.4 (2017-02-27)
------------------
* Add offline_state param to state helper
  We want to be able to keep apps stopped when offline, but keep old
  behavior as default.
* Contributors: Matt Vollrath

1.7.3 (2017-02-26)
------------------

1.7.2 (2017-02-24)
------------------

1.7.1 (2017-02-23)
------------------
* Only apply Chrome kiosk workaround in --kiosk mode
  Don't risk breaking non--kiosk Chrome window placement.
* Add optional support for Chrome --kiosk windows
  Using this workaround all the time was breaking other apps, so only use
  it when launching a browser.
* Contributors: Matt Vollrath

1.7.0 (2017-02-22)
------------------
* commenting out flipping tests
* Contributors: Jacob Minshall

1.6.5 (2017-02-08)
------------------

1.6.4 (2017-02-07)
------------------

1.6.3 (2017-02-03)
------------------
* Emit initial state for on_offline_message  (`#327 <https://github.com/endpointcorp/lg_ros_nodes/issues/327>`_)
  * Renamed connectivity_topic to offline topic
  * Use initial state as defaul on_offline_state state
  * Update tests
  * pep8
  * Publish state message only on change online/ofline state
* Wait until browser gets spawned
* Made proper logic for rc25
* PEP8 fix
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz

1.6.2 (2017-01-25)
------------------
* Fixed influx exception writing
* use inintial state as default state for offline mode
* Fix PEP8 errors
* Add required_param helper with tests
* Contributors: Matt Vollrath, Wojciech Ziniewicz, kiselev-dv

1.6.1 (2017-01-12)
------------------
* fix error in director state setter
* Contributors: Will Plaut

1.6.0 (2016-12-23)
------------------
* fixing the director state setter
* now ignoring stop presentations scene
* Made managed adhoc browser' tests' setUp and tearDown methods great a (`#319 <https://github.com/endpointcorp/lg_ros_nodes/issues/319>`_)
  * Made managed adhoc browser' tests' setUp and tearDown methods great again
  * Probably fixed lg_stats tests
  * Made all ros nodes voluntarily submit exceptions to influx
  * Initial version of lg_Ros_nodes base
  * updated docs for lg_ros_nodes_base
  * Ping CI
  * Ping CI
  * Proper name for dockerfile
  * Dont clean up stuff - jenkins will do it
  * Wait 2 secs to turn into active
  * Made changes to lg_activity tests to be less load susceptible
  * Poll tracker until becomes inactive
  * Another try to poll activity status
  * Even more tests refactoring
  * Remove unnecessary asserts
  * Let's just not
  * Increase message emission grace time
  * Removed even more unncecessary asserts
  * Fix wrong var during exception handling
  * Possible breakage fix
* Contributors: Jacob Minshall, Will Plaut, Wojciech Ziniewicz

1.5.26 (2016-12-21)
-------------------
* Disabled tests temporarily
* Proper extended activity tracker test
* Poll activities to know their state before assert
* Add more time for the rosbridge param test
* More debug for browser preloading breaker test
* lg_common: helpers: fixing issues in the director listener
* lg_common: helpers: generic state setter from director messages
* Contributors: Jacob Minshall, Will Plaut, Wojciech Ziniewicz

1.5.25 (2016-12-14)
-------------------
* Added influx respawn influx handlers
* Contributors: Wojciech Ziniewicz

1.5.24 (2016-11-30)
-------------------

1.5.23 (2016-11-30)
-------------------

1.5.22 (2016-11-21)
-------------------

1.5.21 (2016-11-17)
-------------------

1.5.20 (2016-11-17)
-------------------

1.5.19 (2016-11-16)
-------------------
* Issue/end point corp/lg chef`#1031 <https://github.com/EndPointCorp/lg_ros_nodes/issues/1031>`_ (`#317 <https://github.com/EndPointCorp/lg_ros_nodes/issues/317>`_)
  * Use method for evaluation of active sources
  * Added new strategy for counting sessions
  * pinging PR build
* Contributors: Wojciech Ziniewicz

1.5.18 (2016-11-14)
-------------------

1.5.17 (2016-11-11)
-------------------

1.5.16 (2016-11-07)
-------------------

1.5.15 (2016-11-04)
-------------------
* PEP8
* Contributors: Wojciech Ziniewicz

* PEP8
* Contributors: Wojciech Ziniewicz

1.5.14 (2016-11-04)
-------------------
* Features/screenshots (`#312 <https://github.com/EndPointCorp/lg_ros_nodes/issues/312>`_)
  * screenshots node
  * fixed setup.py
  * fixed setup.py
  * fixed CMakeList
  * Add tests
  * Fix tests
  * Fix tests
  * Fix tests
  * Fix access flags
  * Add readme, fix version, fix paths, fix test
  * PEP8
  * PEP8 and fixes
  * Fixed test
* Contributors: Dmitry Kiselev

1.5.13 (2016-11-04)
-------------------
* Added offliner to browser pool tests as a dep
* Fixed initial state of lg_offliner when configured to run multiple checks and added offliner service dependency to adhoc browser pool
* Contributors: Wojciech Ziniewicz

1.5.12 (2016-11-03)
-------------------
* hide chrome warning of no-sandbox being insecure
* set kiosk=True in test file adhocbrowser creation
  The default should be true, but because this is a message type, we need
  to manually set it. Inside the adhoc browser we always set kiosk to true
  or whatever the user passed to the director message. We never rely on
  the default when creating a new instance of AdhocBrowser()
* Contributors: Jacob Minshall

1.5.11 (2016-11-03)
-------------------
* add support for kiosk mode setting through activity_config
* remove write_log_to_file call
* Added initial docker version for lg (`#309 <https://github.com/endpointcorp/lg_ros_nodes/issues/309>`_)
  * Added initial docker version for lg
  * PEP8
  * Converted from ros:indigo to ubuntu
  * Nvidia
  * X support for OSX and Linux and other goodies
  * Run Xvfb during tests
  * Added no-sandbox to disable debugging
* fix soft relaunches in adhoc browser
* Contributors: Jacob Minshall, Wojciech Ziniewicz

1.5.10 (2016-10-31)
-------------------
* Refactored rfreceiver and fixed tests for chrome url monitor
* More tests fixing
* PEP8 and tests refactoring
* Contributors: Wojciech Ziniewicz

1.5.9 (2016-10-28)
------------------
* add the new lib directories
* Contributors: Jacob Minshall

1.5.8 (2016-10-27)
------------------

1.5.7 (2016-10-27)
------------------
* Better cleanup in tests
* Contributors: Wojciech Ziniewicz

1.5.6 (2016-10-26)
------------------
* Forgot to sleep
* Converted dumb waits to something more robust
* Test fixing
* Made AdhocBrowser data structure for defining gogoel chrome version compatible with ros cms data
* Contributors: Wojciech Ziniewicz

1.5.5 (2016-10-26)
------------------
* Match Chrome unstable window instance names
  There are ever-changing variations, but the path to the tmp_dir is always present and unique.
* Contributors: Matt Vollrath

1.5.4 (2016-10-25)
------------------
* Fix adhoc browser test
* Contributors: Matt Vollrath

1.5.3 (2016-10-25)
------------------
* Revamp delays in ad hoc browser pool
  * Remove bad delays from ManagedApplication
  * Converge window before setting ProcController goal state
  * Fix some other cruft
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.5.2 (2016-10-19)
------------------
* Better readiness logging and forceful activation
* Contributors: Wojciech Ziniewicz

1.5.1 (2016-10-19)
------------------

1.5.0 (2016-10-19)
------------------
* Fix typo in ros_window_ready extension
* Remove infinite cycle in ros_window_ready extension logging
* JS extensions logs cleanup
* Contributors: kiselev-dv

1.4.19 (2016-10-18)
-------------------
* Parametrized hide and destroy delay and PEP8ized
* Revert "Revert "Added delay""
  This reverts commit c6df1f7e3a3e9a3e6d07d255648a468c54ec5075.
* Fix typo
* PEP8 and stuffz
* Some hokeypokey
* Revert "Added delay"
  This reverts commit 690661968ed22ea648ff2f2b0d2fd2426312ea7f.
* Small amendments
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Added delay
* Work towards `#295 <https://github.com/EndPointCorp/lg_ros_nodes/issues/295>`_
  - added ReadinessHandbrake class
  - made new try_to_become_ready method with some idempotency
  - added tests
* Contributors: Galaxy Admin, Wojciech Ziniewicz

1.4.18 (2016-10-17)
-------------------

1.4.17 (2016-10-13)
-------------------

1.4.16 (2016-10-13)
-------------------
* Amended test waits
* Contributors: Wojciech Ziniewicz

1.4.15 (2016-10-13)
-------------------
* Current url extension fixes
* PEP8
* Initial state setting tests and tuning
  - added scripts/relaunch_test.sh to see test if chrome comes up after
  relaunch
  - added adhoc_browser procedure for checking if all topics are connected
* Fixed state switching in uscs service
* Fix adhoc_browser test
* Fix adhoc_browser test
* Fix adhoc_browser test
* Fix adhoc_browser test
* Fix adhoc_browser test
* Fix adhoc_browser test
* Fix adhoc_browser test
* Fix adhoc_browser test
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into browser_url
  Conflicts:
  lg_common/src/lg_common/adhoc_browser_director_bridge.py
* Changed adhoc_browser test
* Changed adhoc_browser test
* Parse string and obj extensions in director bridge
* Fix extension publishing message type
* Fix extensions parsing in director bridge
* Fixed extension (topic name and roslib initialization)
  Fixed url normalization in service
  Fixed typo in service
* fix syntax
* Merge json manualy
* Fix topic
* Refactored to use browsers service for url tracking
* Ros service for curent url
* Ros sceleton
* Extension
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz, kiselev-dv

1.4.14 (2016-10-11)
-------------------

1.4.13 (2016-10-10)
-------------------
* Test precedence matters
* Refactored adhoc browser tests and fixed a bug
* properly set the uscs message response
* fix route_touch_to_viewports
  No longer filter by activity_type.
* Fixed preloading logic
* Contributors: Jacob Minshall, Wojciech Ziniewicz, wojciech ziniewicz

1.4.12 (2016-10-07)
-------------------
* TEst fix + pep8
* Readiness changes
  - made director service wait for readiness node before sending messages
  - made readiness evaluate total number of browsers using director topic
  instead of non-comprehensive common browser topic
* Contributors: Wojciech Ziniewicz, wojciech ziniewicz

1.4.11 (2016-10-06)
-------------------
* Changes t spacenav globe and ros window ready extension
* amended the cmd args
* Contributors: Wojciech Ziniewicz, wojciech ziniewicz

1.4.10 (2016-10-06)
-------------------
* Changed attrib name for retrieving command line args
* Work for `#296 <https://github.com/EndPointCorp/lg_ros_nodes/issues/296>`_ (`#299 <https://github.com/EndPointCorp/lg_ros_nodes/issues/299>`_)
* Changed path to extensions attribute
* Contributors: Wojciech Ziniewicz, wojciech ziniewicz

1.4.9 (2016-10-04)
------------------

1.4.8 (2016-10-03)
------------------

1.4.7 (2016-10-03)
------------------
* More changelogs
* Generated changelog
* Implement page urls monitor extension (`#293 <https://github.com/EndPointCorp/lg_ros_nodes/issues/293>`_)
  * Urls monitoring
  * Parse allowed urls config from get args
  * page monitor parameters passing
  * Page urls monitoring: readme, tests and get_args passing
  * Add allowed urls to adhoc browser message
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Revert "REnamed helper method"
  This reverts commit 1b6343469bb20d3fe3bf00a7098063f78c904131.
  * Tests amendment and PEP8
  * Added missing files
  * Amending tests to match ros_window_ready new bahavior
  * More amendments to ros_window_ready
  * Fixed test roslaunch files
  * Fixed log string eval and uscs tests
  * Amended tests
* REnamed helper method
* Added rosbridge deps for adhoc browser
* Made lg_mirror activate itself slightly later with custom preload message
* Fixed verbosity of USCS
* fixed undefined constant
* Fix null pointer
* Switch run_at to document start for ros_window_ready extension
* Switch run_at to document start for ros_window_ready extension
* Repeat window ready message once a sec.
* PEP8 and some reduntant stuffz removal
* Onboard test coverage
  - added onboard_router symlink and made it deployable
  - amended tests to cover mirroring only (it's the only one that we want
  to support)
  - amended tests slightly to go green
* Onboard and touch routing
  - added TDD stuffz for onboard router
  - factored out shared methods from lg_mirror to helpers
  - created test_helpers for message and window generation
  - created onboard_router ros node
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz, Zdenek Maxa, kiselev-dv

* Generated changelog
* Implement page urls monitor extension (`#293 <https://github.com/EndPointCorp/lg_ros_nodes/issues/293>`_)
  * Urls monitoring
  * Parse allowed urls config from get args
  * page monitor parameters passing
  * Page urls monitoring: readme, tests and get_args passing
  * Add allowed urls to adhoc browser message
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Revert "REnamed helper method"
  This reverts commit 1b6343469bb20d3fe3bf00a7098063f78c904131.
  * Tests amendment and PEP8
  * Added missing files
  * Amending tests to match ros_window_ready new bahavior
  * More amendments to ros_window_ready
  * Fixed test roslaunch files
  * Fixed log string eval and uscs tests
  * Amended tests
* REnamed helper method
* Added rosbridge deps for adhoc browser
* Made lg_mirror activate itself slightly later with custom preload message
* Fixed verbosity of USCS
* fixed undefined constant
* Fix null pointer
* Switch run_at to document start for ros_window_ready extension
* Switch run_at to document start for ros_window_ready extension
* Repeat window ready message once a sec.
* PEP8 and some reduntant stuffz removal
* Onboard test coverage
  - added onboard_router symlink and made it deployable
  - amended tests to cover mirroring only (it's the only one that we want
  to support)
  - amended tests slightly to go green
* Onboard and touch routing
  - added TDD stuffz for onboard router
  - factored out shared methods from lg_mirror to helpers
  - created test_helpers for message and window generation
  - created onboard_router ros node
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz, Zdenek Maxa, kiselev-dv

* Implement page urls monitor extension (`#293 <https://github.com/EndPointCorp/lg_ros_nodes/issues/293>`_)
  * Urls monitoring
  * Parse allowed urls config from get args
  * page monitor parameters passing
  * Page urls monitoring: readme, tests and get_args passing
  * Add allowed urls to adhoc browser message
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Tests for allowed urls message passing
  * Revert "REnamed helper method"
  This reverts commit 1b6343469bb20d3fe3bf00a7098063f78c904131.
  * Tests amendment and PEP8
  * Added missing files
  * Amending tests to match ros_window_ready new bahavior
  * More amendments to ros_window_ready
  * Fixed test roslaunch files
  * Fixed log string eval and uscs tests
  * Amended tests
* REnamed helper method
* Added rosbridge deps for adhoc browser
* Made lg_mirror activate itself slightly later with custom preload message
* Fixed verbosity of USCS
* fixed undefined constant
* Fix null pointer
* Switch run_at to document start for ros_window_ready extension
* Switch run_at to document start for ros_window_ready extension
* Repeat window ready message once a sec.
* PEP8 and some reduntant stuffz removal
* Onboard test coverage
  - added onboard_router symlink and made it deployable
  - amended tests to cover mirroring only (it's the only one that we want
  to support)
  - amended tests slightly to go green
* Onboard and touch routing
  - added TDD stuffz for onboard router
  - factored out shared methods from lg_mirror to helpers
  - created test_helpers for message and window generation
  - created onboard_router ros node
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz, Zdenek Maxa, kiselev-dv

1.4.6 (2016-09-28)
------------------

1.4.5 (2016-09-21)
------------------

1.4.4 (2016-09-21)
------------------
* Amended tests to honor new behavior of uscs service
* WE're not using underscores anymore
* Pep8ized code
* Activity, USCS service and mirror amendments
  - made mirror re-publish messages for touch receiver after earlier
  initial state subscription
  - made activity send initial state again and uscs service ignore it
* Remove extension
* rename extension
* rename
* Extension for tactile smooth loading
* Updated readme with rosbridge parameters description
* Various preloading fixes
  - added Activity service definition
  - amended logging
  - made unhiding not destroy browsers badly
* Read get parameters for rosbridge and ros_window_name from history
* Fixes for initial scene handling by lg_mirror and activity service for lg_Activity
* Pass rosbridge connection params from rosparam via adhocbrowserspool get_args
  Fix build
  Add comments for further refactor
  Add test for adhock_browser_pool
  Add test for adhoc_browser_pool
  Actual test for parameters passing
  Made tet for adhoc browser pool
  Made tet for adhoc browser pool
  Fix tests
  Fix tests
  Fix tests
  Fix tests
  Fix tests
  Fix tests
  More debug output for tests
  More debug output for tests
  More debug output for tests
  Print stdout for rostest
  Print stdout for rostest
  Print stdout for rostest
  Print stdout for rostest
  Print stdout for rostest
  Tests for rosbridge connection
  More tests
  More tests
  More tests
  Reverted test_runner
* Revert "Add default rosbridge config for ros window ready extension"
  This reverts commit 4928aa929ffba2fe1bc0af7b813b70ffac72b229.
* Add default rosbridge config for ros window ready extension
* Switch ros_window_ready extension to use ssl connection by default
* Contributors: Wojciech Ziniewicz, kiselev-dv

1.4.3 (2016-09-12)
------------------

1.4.2 (2016-09-12)
------------------

1.4.1 (2016-09-12)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Amended docs and extensions root
* Contributors: Wojciech Ziniewicz

1.4.0 (2016-09-06)
------------------
* initial state setting of ros nodes (`#270 <https://github.com/endpointcorp/lg_ros_nodes/issues/270>`_)
  * initial state setting of ros nodes
  * Made new initial vars and mechanisms as a work towards completion of `#274 <https://github.com/endpointcorp/lg_ros_nodes/issues/274>`_
  * Made new initial vars and mechanisms as a work towards completion of `#274 <https://github.com/endpointcorp/lg_ros_nodes/issues/274>`_
  * Added test suite and functoinality for uscs service
  * Fixed a typo
  * Added USCS service to kmlsync tests
  * DRYed out uscs code and pep8 fixes
  * Removing wait_for_service dependency
  * Added test coverage for setting initial state for adhoc_browser_pool `#165 <https://github.com/endpointcorp/lg_ros_nodes/issues/165>`_
  * import generic message in test
* Contributors: Jacob Minshall

1.3.31 (2016-09-01)
-------------------
* Refactored adhoc browser pool housekeeping
* Contributors: Galaxy Admin

1.3.30 (2016-08-31)
-------------------

1.3.29 (2016-08-31)
-------------------
* synced broken changelogs
* Raised some timeouts again
* Longer timeout for custom callback
* Formatted teh changelog
* Dont rely on postponed removal - shouldnt be tested
* More timeouts!
* Making custom preloading event better again
* Added Xvfb to docker tests
* Split test cases to separate defs and made Xvfb enabled in Docker
* Changed maps.google.com slowness to something local and fast and amended some tests
* Contributors: Wojciech Ziniewicz

* Raised some timeouts again
* Longer timeout for custom callback
* Formatted teh changelog
* Dont rely on postponed removal - shouldnt be tested
* More timeouts!
* Making custom preloading event better again
* Added Xvfb to docker tests
* Split test cases to separate defs and made Xvfb enabled in Docker
* Changed maps.google.com slowness to something local and fast and amended some tests
* Contributors: Wojciech Ziniewicz

1.3.28 (2016-08-26)
-------------------
* added smooth transitions functionality #251 `https://github.com/EndPointCorp/lg_ros_nodes/issues/251`
* added support for loading chrome extensions in AdhocBrowser
* added support for adding command line arguments
* added support for using custom chrome binaries
* Contributors: Wojciech Ziniewicz

1.3.27 (2016-08-23)
-------------------
* Fix whitespace error in adhoc browser test
* Add garbage collection test for ManagedApplication
  This is part of `#262 <https://github.com/EndPointCorp/lg_ros_nodes/issues/262>`_
* Add close() method to ManagedApplication
* Remove _signal_proc from ManagedApplication
  This feature is no longer needed and never worked anyway.
* Contributors: Matt Vollrath

1.3.26 (2016-08-15)
-------------------
* fix log messages for soft relaunches
* add soft relaunch support for all media assets
  browser graphics, browser videos, and mplayer videos supported.
* Contributors: Jacob Minshall

1.3.25 (2016-08-12)
-------------------
* better detection of new and old assets
* Added geometry checks for browser persistence and failed to cover
  edgecase http://d.pr/i/1it1J
* Contributors: Galaxy Admin, Wojciech Zieniewicz

1.3.24 (2016-08-12)
-------------------

1.3.23 (2016-08-09)
-------------------

1.3.22 (2016-08-09)
-------------------
* generating changelogs to satisfy jenkins lg_ros_nodes_deb_builds_master, touch: `#113 <https://github.com/EndPointCorp/lg_ros_nodes/issues/113>`_
* unpack_activity_sources extensions
  -corrected function's docstrings
  -added unittets (none previously)
  -implemented single value for values stratedy (needed for `#113 <https://github.com/EndPointCorp/lg_ros_nodes/issues/113>`_),
  including a unittest
  -narrowed exception clauses
  -touch: `#187 <https://github.com/EndPointCorp/lg_ros_nodes/issues/187>`_
* Contributors: Zdenek Maxa

* unpack_activity_sources extensions
  -corrected function's docstrings
  -added unittets (none previously)
  -implemented single value for values stratedy (needed for `#113 <https://github.com/EndPointCorp/lg_ros_nodes/issues/113>`_),
  including a unittest
  -narrowed exception clauses
  -touch: `#187 <https://github.com/EndPointCorp/lg_ros_nodes/issues/187>`_
* Contributors: Zdenek Maxa

1.3.21 (2016-08-03)
-------------------
* Add last_uscs service to lg_common
* Contributors: Szymon Lipinski

1.3.20 (2016-07-29)
-------------------

1.3.19 (2016-07-29)
-------------------
* Fix whitespace in managed_browser.py
* Contributors: Matt Vollrath

1.3.18 (2016-07-28)
-------------------
* Ad hoc browser persistence round deux
* Contributors: Matt Vollrath

1.3.17 (2016-07-27)
-------------------
* Revert "Persist ad hoc browser assets across scenes"
  This was breaking ad hoc browsers.
* Contributors: Matt Vollrath

1.3.16 (2016-07-26)
-------------------

1.3.15 (2016-07-26)
-------------------
* Clear browser tmpdir on shutdown
* Persist ad hoc browser assets across scenes
  URL's are now encoded in consistent order.
* Contributors: Matt Vollrath

1.3.14 (2016-07-25)
-------------------
* Fix ManagedBrowser race conditions
  * Add post_init() for ManagedApplication
  * Add add_respawn_handler and add_state_handler to ManagedApplication
  * Bring back set_state() locking
  * Lock AdhocBrowserPool message handling
  * Start and stop the Chrome debug relay inside locked set_state()
  * Remove dangerous code from ManagedApplication _handle_respawn()
* Contributors: Matt Vollrath

1.3.13 (2016-07-21)
-------------------
* remove lock from managed application set state
* reclassify loginfo as logdebug
* re-classify logerr to loginfo
* Contributors: Jacob Minshall, Wojciech Zieniewicz

1.3.12 (2016-07-19)
-------------------
* fix syntax error
* implement rosparam for extra chrome logging
* remove chrome's logs by default
  Plus took out an old log message that has been bugging me.
* Comment Chrome window instance rule
  Thanks Dmitry for asking.
* Contributors: Jacob Minshall, Matt Vollrath

1.3.11 (2016-07-15)
-------------------
* set state of managed application on respawn
* set proc to None after wait and set shell=True
  With shell=True we no longer need to have /bin/sh -c be the start of our
  command.
* Removed @minnshalj comment
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into EndPointCorp/lg_chef_860_graphics_loading_timing_issues
  Conflicts:
  lg_common/src/lg_common/adhoc_browser_pool.py
  lg_media/src/lg_media/mplayer_pool.py
* Removed instance updating completely
* Contributors: Galaxy Admin, Jacob Minshall

1.3.10 (2016-07-13)
-------------------
* Fixed `EndPointCorp/lg_chef#859 <https://github.com/EndPointCorp/lg_chef/issues/859>`_
* Fixed browser vid player
* Contributors: Galaxy Admin, Wojciech Ziniewicz

1.3.9 (2016-07-08)
------------------
* Added a close_fds fix for awesome and fixed a logging method exception that was crashing lg_attract_loop
* Clear browser tmpdir on each spawn
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.3.8 (2016-07-06)
------------------

1.3.7 (2016-07-05)
------------------

1.3.6 (2016-07-01)
------------------

1.3.5 (2016-07-01)
------------------

1.3.4 (2016-07-01)
------------------

1.3.3 (2016-06-30)
------------------
* lg_stats and lg_attract_loop amendments (`#246 <https://github.com/EndPointCorp/lg_ros_nodes/issues/246>`_)
  * Made lg_stats and lg_attract_loop verbosity great again. Added default action to lg_attract_loop to make it decent again
  * Planet default
  * URL override for touchscreen
  * Pep8ized tests
* Contributors: Wojciech Ziniewicz

1.3.2 (2016-06-29)
------------------
* Fixed debug in lg_stats
* Contributors: Wojciech Ziniewicz

1.3.1 (2016-06-28)
------------------
* refactored x_available to DRY out code mode
* factor out dependency_available to check_www_dependency
  This DRYs out the code a bunch.
* Contributors: Will Plaut

1.3.0 (2016-06-25)
------------------
* Fix remote debugging port in scripts
* Track pytest dep in lg_common
* Enable GPU rasterization in Chrome by default
* Implement TCPRelay in ManagedBrowser
* Add TCP relay for Chrome debug relay
* Added rosnode name parameter for adhock browser pool (`#234 <https://github.com/EndPointCorp/lg_ros_nodes/issues/234>`_)
  * Add ros_instance_name get parameter for adhoc browsers pool
  * PEP8ized code for `#234 <https://github.com/EndPointCorp/lg_ros_nodes/issues/234>`_
* reverted adhoc_browser_pool.py
* lg_common helpers tests, related to touch: `#193 <https://github.com/EndPointCorp/lg_ros_nodes/issues/193>`_
* Browser fixes (`#232 <https://github.com/EndPointCorp/lg_ros_nodes/issues/232>`_)
  * various ManagedBrowser fixups
  - Fixed browser names for `#145 <https://github.com/EndPointCorp/lg_ros_nodes/issues/145>`_
  - added defult disk_cache_size param of 300mb for `#148 <https://github.com/EndPointCorp/lg_ros_nodes/issues/148>`_
  - added stderr logging to logger pipe
  * Various browser fixes
  - limited browser disk cache size to 300mb `#148 <https://github.com/EndPointCorp/lg_ros_nodes/issues/148>`_
  - parametrized verbosity of browsers `#129 <https://github.com/EndPointCorp/lg_ros_nodes/issues/129>`_
  - made browsers use proper slugs including viewports in their names `#145 <https://github.com/EndPointCorp/lg_ros_nodes/issues/145>`_
  * Pep8ed
  * Pep8'd files
* Use local copy of lg_ros_nodes
* Contributors: Dmitry Kiselev, Matt Vollrath, Wojciech Ziniewicz, Zdenek Maxa

1.2.14 (2016-06-10)
-------------------

1.2.13 (2016-06-10)
-------------------
* mplayer on_finish -> respawn attribute, touch: `#193 <https://github.com/EndPointCorp/lg_ros_nodes/issues/193>`_
* Contributors: Zdenek Maxa

1.2.12 (2016-06-07)
-------------------
* Issue/226 float value lg stats (`#228 <https://github.com/EndPointCorp/lg_ros_nodes/issues/228>`_)
  * lg_stats development
  - added metadata to Event message type
  - added value to all influx measurements
  - made resubmission thread use value of 0.5
  * Fixing tests for lg_stats
  * Pep8'd
* Fix pep8 in lg_common helpers
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.2.11 (2016-06-02)
-------------------
* Catch KeyError in get_activity_config

1.2.10 (2016-05-20)
-------------------
* get_activity_config now in lg_common helpers
* Contributors: Jacob Minshall

1.2.9 (2016-05-20)
------------------

1.2.8 (2016-05-19)
------------------

1.2.7 (2016-05-17)
------------------
* Merge branch 'wip-lg_activity_tests' of github.com:endpointcorp/lg_ros_nodes into development
* fixing pep8
* Merge branch 'development' of github.com:endpointcorp/lg_ros_nodes into wip-lg_activity_tests
  not quite fixed yet
  Conflicts:
  lg_activity/src/lg_activity/activity.py
  lg_activity/test/online/test_tracker.test
  lg_common/src/lg_common/helpers.py
* lg_activity: remove unneeded assignment
* WIP more work in progress
* WIP more wip...
* WIP change msg_type to message_type
  I think it was called both, so now only one is used.
* WIP squash this commit
* Contributors: Galaxy Admin, Jacob Minshall, Wojciech Ziniewicz, Zdenek Maxa

1.2.6 (2016-05-16)
------------------

1.2.5 (2016-05-12)
------------------

1.2.4 (2016-05-10)
------------------
* lg_sv: ignore 'no_activity' scene
  Also don't just check for the first window's activity to check for the
  streetview activity type, check all activities. This will allow us to
  have images overlayed on streetview without running into issues.
* Contributors: Jacob Minshall

1.2.3 (2016-05-06)
------------------
* Generated changelogs
* 1.2.2
* PEP8
* Added count_nonzero strategy for `#208 <https://github.com/EndPointCorp/lg_ros_nodes/issues/208>`_
* Contributors: Wojciech Ziniewicz

1.2.1 (2016-05-03)
------------------

1.2.0 (2016-04-29)
------------------
* pep8 fixes
* Contributors: Jacob Minshall

1.1.50 (2016-04-27)
-------------------
* move new loginfo logging to logdebug
* added log watcher
* fix up logging
  Move some logerrs to log{warn,info} depending on the information being
  logged. Also s/logger.erroror/logger.error/
* Contributors: Jacob Minshall

1.1.49 (2016-04-26)
-------------------
* Fixed tests:
  - renamed files to reflect new functionality (new strategies) - tests
  coverage is missing for non-default ones
  - amended some code I wrongly added to meat
  - pep8'ized code
* Fixed tests for `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* lg_stats part 2
  - re-thinked activity sources parsing - lg_activity tests need to be
  written to make sure its not broken
  - moved count and average processors to background tasks like
  resubmitters - good idea by @zdenekmaxa
  - added `measurement` message field and moved attribute mapping so that
  ROS topic are independent from measurment names
* Some docs amendments
* lg_stats strategies and activity sources:
  - added support for nested slots value extraction
  - refactored lg_activity to use shared helper for the above
  - removed cruft for strategies - replaced with proper strategies
  - added support for count and average
  - didnt test it yet - havent amended tests to resemble new functionality
  yet
* Contributors: Wojciech Ziniewicz

1.1.48 (2016-04-20)
-------------------

1.1.47 (2016-04-15)
-------------------

1.1.46 (2016-04-15)
-------------------
* fix up changelogs
* Contributors: Jacob Minshall

1.1.45 (2016-04-14)
-------------------

1.1.44 (2016-04-14)
-------------------

1.1.43 (2016-04-14)
-------------------

1.1.42 (2016-04-14)
-------------------
* updated changelogs for new release
* restart browser on soft relaunch
* restart earth process on soft relaunch
* softrelaunch initial work
* Contributors: Jacob Minshall, Zdenek Maxa

* use rosunit to run offline tests, touch: `#195 <https://github.com/EndPointCorp/lg_ros_nodes/issues/195>`_
* restart browser on soft relaunch
* restart earth process on soft relaunch
* softrelaunch initial work
* Contributors: Jacob Minshall, Zdenek Maxa

1.1.41 (2016-04-13)
-------------------
* Generated changelogs while preparing for new release
* Update managed_browser.py
  Add `--enable-webgl` `--ignore-gpu-blacklist` for managed browser
* Contributors: Dmitry Kiselev, Zdenek Maxa

* Update managed_browser.py
  Add `--enable-webgl` `--ignore-gpu-blacklist` for managed browser
* Contributors: Dmitry Kiselev

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
* Fix logger.error method names
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
