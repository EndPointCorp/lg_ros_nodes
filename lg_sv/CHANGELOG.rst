^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_sv
^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2020-05-27)
------------------

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

3.3.15 (2021-12-09)
-------------------

3.3.14 (2021-11-23)
-------------------

3.3.13 (2021-09-21)
-------------------

3.9.1 (2024-12-17)
------------------

3.9.3 (2025-04-02)
------------------
* Write sys.deps for all packages
* Contributors: Matt Vollrath

3.9.5 (2025-04-08)
------------------

3.9.4 (2025-04-08)
------------------
* 3.9.3
* Changelogs
* Write sys.deps for all packages
* Contributors: Matt Vollrath

3.9.10 (2025-04-15)
-------------------

3.9.9 (2025-04-15)
------------------
* director messages don't make more director messages
* Contributors: Jacob Minshall

3.9.8 (2025-04-14)
------------------
* Use get_package_path
  Cleanup some unused rospkg imports.
* Contributors: Matt Vollrath

3.9.7 (2025-04-11)
------------------

3.9.6 (2025-04-09)
------------------
* 3.9.5
* bump
* 3.9.4
* bump
* 3.9.3
* Changelogs
* Write sys.deps for all packages
* Contributors: Jacob Minshall, Matt Vollrath

3.9.2 (2024-12-23)
------------------
* fix up javascript to look for assets location of data too
* importing
* one more place to change
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

3.8.1 (2023-10-06)
------------------

3.8.0 (2023-05-25)
------------------
* Merge pull request `#416 <https://github.com/endpointcorp/lg_ros_nodes/issues/416>`_ from EndPointCorp/quiet_logs_some
  log quieting
* big logging changes
* Merge branch 'master' of github.com:endpointcorp/lg_ros_nodes into quiet_logs_some
* log quieting
* Contributors: Jacob Minshall

3.7.2 (2023-04-27)
------------------

3.7.1 (2023-04-06)
------------------

3.7.0 (2023-03-30)
------------------

3.6.0 (2022-11-22)
------------------

3.5.5 (2022-09-09)
------------------

3.5.4 (2022-08-10)
------------------

3.5.3 (2022-05-27)
------------------
* Fixed the streetview issues caused by duplicate CORS headers
* Contributors: Kannan Ponnusamy

3.5.2 (2022-03-29)
------------------

3.5.1 (2022-03-24)
------------------
* 3.5.0
* bump changelogs
* 3.4.1
* Changelogs
* 3.4.0
* bump changelogs
* Merge branch 'master' of github.com:endpointcorp/lg_ros_nodes into fake_spacenav
* 3.3.16
* bump changelogs
* 3.3.15
* bump changelogs
* 3.3.14
* changelogs
* 3.3.13
* bump changelogs
* Contributors: Jacob Minshall, Matt Vollrath, Will Plaut

3.3.12 (2021-08-10)
-------------------

3.3.11 (2021-07-23)
-------------------

3.3.10 (2021-07-22)
-------------------
* Normalize ManagedAdhocBrowser layers
* Contributors: Matt Vollrath

3.3.9 (2021-07-20)
------------------

3.3.8 (2021-01-29)
------------------

3.3.7 (2021-01-14)
------------------

3.3.6 (2021-01-06)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: dkiselev

3.3.5 (2020-12-28)
------------------

3.3.4 (2020-12-11)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: dkiselev

3.3.3 (2020-12-10)
------------------
* metedata search broken due to undocumented API being used
  Doesn't seem to be used anywhere else, need to get API key and change
  url to https://maps.googleapis.com/maps/api/streetview/metadata?location={lat}%2C{lon}&radius={radius}&key={api_key}
  grabbed from here: https://stackoverflow.com/questions/42019281/getting-street-view-panoroma-id-from-lat-long-coordinates
* Contributors: Will Plaut

3.3.2 (2020-12-09)
------------------

3.3.1 (2020-10-29)
------------------
* Remove whitespace to fix the pep8
* Contributors: Kannan Ponnusamy

3.3.0 (2020-10-21)
------------------
* Merge pull request `#431 <https://github.com/EndPointCorp/lg_ros_nodes/issues/431>`_ from EndPointCorp/topic/fix_streetview
  Fixes to consolidate the streetview experience
* More cleanup
* Fix for the streetview navigation problem
  - Tested the changes on the RDTN box and it's working! Yay!
* Remove more references to streetview_old
  - This code block somehow intercepts the director message and lead to
  the problem with the open source version of viewer
* Remove more references of streetview_old
* Remove references of streetview_old from the ros node server
* Contributors: Kannan Ponnusamy

3.2.9 (2020-09-21)
------------------
* Merge pull request `#429 <https://github.com/EndPointCorp/lg_ros_nodes/issues/429>`_ from EndPointCorp/topic/use_opensource_sv_viewer
  Use opensource streetview viewer by default
* Use opensource sv viewer by default
* Contributors: Kannan Ponnusamy

3.2.8 (2020-08-27)
------------------
* Add large viewport hack to streetview
  Use scale tricks to get a legal canvas size.
  Add parameters:
  * show_api_links : show the built-in streetview links
  * large_viewport_hack: for window dimensions > 16384(?)
* Contributors: Matt Vollrath

3.2.7 (2020-08-25)
------------------

3.2.6 (2020-07-09)
------------------

3.2.5 (2020-07-06)
------------------
* Merge pull request `#426 <https://github.com/EndPointCorp/lg_ros_nodes/issues/426>`_ from EndPointCorp/topic/sv_user_photospheres
  Chrome extension and ROS node changes to fix the problems with streetview interface
* Removed hard-coded path and used rospkg to get it
* Initial version of chrome extension and ros node changes to fix the
  streetview problems `#425 <https://github.com/EndPointCorp/lg_ros_nodes/issues/425>`_
* Contributors: Kannan Ponnusamy

3.2.4 (2020-05-29)
------------------
* lg_sv: launcher change to correctly set suffix
* Contributors: Will Plaut

3.2.3 (2020-05-28)
------------------

3.2.2 (2020-05-28)
------------------

3.2.1 (2020-05-27)
------------------
* lg_sv: unique slug names with uuid (`#393 <https://github.com/EndPointCorp/lg_ros_nodes/issues/393>`_)
  * lg_sv: unique slug names with uuid
  * launcher.py: use viewport instead of uuid
  * sv: account for empty viewports
* 3.2.0
* BSPWM
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Galaxy Admin, Neil Elliott, Will Plaut, dkiselev

3.1.12 (2020-05-04)
-------------------

3.1.11 (2020-05-01)
-------------------
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

3.1.6 (2020-01-27)
------------------

3.1.5 (2020-01-24)
------------------

3.1.4 (2020-01-24)
------------------

3.1.3 (2020-01-20)
------------------
* Merge pull request `#413 <https://github.com/EndPointCorp/lg_ros_nodes/issues/413>`_ from EndPointCorp/topic/fix_js_msg_types
  lots of missing msg/srv updates in js files
* lots of missing msg/srv updates in js files
* Contributors: Will Plaut

3.1.2 (2020-01-10)
------------------

3.1.1 (2020-01-08)
------------------

3.1.0 (2020-01-06)
------------------
* Merge pull request `#411 <https://github.com/EndPointCorp/lg_ros_nodes/issues/411>`_ from EndPointCorp/topic/msg_cleanup
  Topic/msg cleanup
* cleanup deps with catkin_lint
* Merge branch 'master' into fix_tests
* Update CMake and package xmls
* Remove msg creation from orig pkgs
* Update service imports
* update import paths everywhere
* Contributors: Matt Vollrath, Will Plaut

3.0.2 (2019-11-06)
------------------

3.0.1 (2019-11-06)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: dkiselev

3.0.0 (2019-10-31)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/python_tree
  Conflicts:
  rosbridge_library/CHANGELOG.rst
  rosbridge_library/package.xml
  rosbridge_server/CHANGELOG.rst
  rosbridge_server/package.xml
* set python executable for tests
* 2to3 all of it
* Merge branch 'topic/python_tree' of github.com:EndPointCorp/lg_ros_nodes into topic/python_tree
* fix weird shebangs
* python 2 shebang to 3
* Contributors: Will Plaut

2.0.18 (2019-10-11)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/image_checker
* Contributors: Will Plaut

2.0.17 (2019-09-11)
-------------------

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

2.0.9 (2019-07-17)
------------------

2.0.8 (2019-07-08)
------------------
* Clean up some lg_sv client topic subs
* Add multi-viewport support for sv launcher
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

1.20.1 (2019-05-21)
-------------------

1.20.0 (2019-05-15)
-------------------
* Fix missing arg in SV server test
* PEP8 sweep
* Fix jslint errors
* reload sv_old on ros connection lost
* reinitialize on ros close
* Contributors: Dmitry Kiselev, Matt Vollrath

1.19.16 (2019-05-14)
--------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/transparent_images
* reinitialize on ros close
* Contributors: Dmitry Kiselev, Galaxy Admin

1.19.15 (2019-04-29)
--------------------
* Reload sv panoviewer if ros connection lost
* Contributors: Dmitry Kiselev

1.19.14 (2019-04-26)
--------------------

1.19.13 (2019-04-25)
--------------------

1.19.12 (2019-03-25)
--------------------

1.19.11 (2019-03-20)
--------------------

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
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
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
* lg_sv: make idle_time_until_snap a rosparam
* Contributors: Will Plaut

1.18.21 (2018-12-05)
--------------------

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
* Use new global ROS param from chef node definition.
* Contributors: Adam Vollrath

1.18.15 (2018-08-24)
--------------------
* Hide Street View Windows instead of closing.
* Don't kill old browser for SV
* Contributors: Adam Vollrath, Dmitry Kiselev

1.18.14 (2018-07-18)
--------------------

1.18.13 (2018-06-22)
--------------------
* Log map api key
* Add map_api_key to launcher script
* Load google maps API dynamically
* Contributors: Dmitry Kiselev

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

1.18.7 (2018-05-04)
-------------------

1.18.6 (2018-05-03)
-------------------

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

1.17.6 (2017-11-15)
-------------------

1.17.5 (2017-11-14)
-------------------
* changing these that no one uses...
* Contributors: Jacob Minshall

1.17.4 (2017-11-10)
-------------------
* using local version of libs
* Contributors: Jacob Minshall

1.17.3 (2017-11-07)
-------------------

1.17.2 (2017-11-06)
-------------------

1.17.1 (2017-10-12)
-------------------

1.17.0 (2017-10-06)
-------------------

1.16.1 (2017-08-17)
-------------------

1.16.0 (2017-08-17)
-------------------

1.15.0 (2017-08-07)
-------------------

1.14.2 (2017-08-02)
-------------------
* condition for panovideos / panoimages
* Contributors: Jacob Minshall

1.14.1 (2017-07-17)
-------------------
* fix up panoid by prepending F: in certain conditions
* Contributors: Jacob Minshall

1.14.0 (2017-07-14)
-------------------
* pep8 fixes
* liten in on the streetview_old topics
* remove the version constraint for the api version
* just an if statement...
  This will require both of our streetview viewers to be on any system.
  Also, I'm not sure our streetview viewer actually works with long form
  panoids anymore.
* Contributors: Jacob Minshall

1.13.5 (2017-06-29)
-------------------

1.13.4 (2017-06-13)
-------------------

1.13.3 (2017-05-31)
-------------------

1.13.2 (2017-05-23)
-------------------

1.13.1 (2017-05-19)
-------------------

1.13.0 (2017-05-19)
-------------------
* Use lg_twister for Earth and Street View
* Contributors: Matt Vollrath

1.12.5 (2017-05-11)
-------------------

1.12.4 (2017-05-11)
-------------------

1.12.3 (2017-05-03)
-------------------

1.12.2 (2017-04-26)
-------------------

1.12.1 (2017-04-24)
-------------------

1.12.0 (2017-04-20)
-------------------

1.11.4 (2017-04-06)
-------------------
* proper setting of the inverted POV for
* Contributors: Jacob Minshall

1.11.3 (2017-03-31)
-------------------
* panoviewer now listens in on director messages
* Contributors: Jacob Minshall

1.11.2 (2017-03-31)
-------------------
* fixes for non-streetview issues
* Contributors: Jacob Minshall

1.11.1 (2017-03-28)
-------------------
* fix for panoviewer
  The panoviewer currently uses a different schema for storing the assets.
  They live in the assets array still.
* Contributors: Jacob Minshall

1.11.0 (2017-03-27)
-------------------
* gjslint fixes
* pep8 fix
* lg_sv: removing un-needed logerrs
* lg_sv: new publisher for tests to translate director->panoid
* Merge branch 'master' of github.com:endpointcorp/lg_ros_nodes into topic/sv_on_director
  Conflicts:
  lg_sv/scripts/server.py
* fixed nearbypanos for director messages
* initial panos are now found via /panoviewer/panoid_state
* attempt to fix nearby panos
* initial sv on director message
  Nearby panos broken when a director scene is published
* Contributors: Jacob Minshall

1.10.2 (2017-03-24)
-------------------

1.10.1 (2017-03-23)
-------------------

1.10.0 (2017-03-23)
-------------------
* Move panovideo to its own package
* Don't create a Map in lg_sv webapp
* Added exception handling during relaunches for lg_replay and lg_sv (`#345 <https://github.com/EndPointCorp/lg_ros_nodes/issues/345>`_)
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.9.1 (2017-03-20)
------------------

1.9.0 (2017-03-20)
------------------
* Add Earth background nav disable to panovideosync
* Contributors: Matt Vollrath

1.8.0 (2017-03-09)
------------------
* Update panovideo .launch
* Add clockAddr arg to panovideo webapp
* Add panovideo transform360 script for posterity
* Update panovideo webapp
  * Major refactor with improvements for video synchronization.
  * Use ES6.
  * Add support for cubic panoramic videos via transform360.
* Add tick_rate param for lg_sv server
* Use uWS sync distributor for panovideo
  Also, interpolate master time in the webapp.
* Contributors: Matt Vollrath

1.7.11 (2017-03-03)
-------------------
* Separate panoid_state Service per server_type
  Fix servers racing to own the streetview panoid service.
* Test lg_sv PanoId state
* Contributors: Matt Vollrath

1.7.10 (2017-03-02)
-------------------
* Legacy yaw offset support for lg_sv launcher
* Contributors: Matt Vollrath

1.7.9 (2017-03-01)
------------------

1.7.8 (2017-03-01)
------------------
* Add tilt param back to lg_sv webapp
* Contributors: Matt Vollrath

1.7.7 (2017-02-28)
------------------
* Added ROS param for showing FPS stats
* Contributors: Wojciech Ziniewicz

1.7.6 (2017-02-27)
------------------
* Load most recent pano at lg_sv webapp startup
  Fixes `#336 <https://github.com/EndPointCorp/lg_ros_nodes/issues/336>`_
* Make lg_sv webapp pixel ratio adjustable
  Gives us a lever to compromise speed/quality.  Default to 1.0 which
  gives no compensation for canvas oversizing.
* Run street view browsers in single window
  Take advantage of local data sharing and consolidated GPU interaction.
* Contributors: Matt Vollrath

1.7.5 (2017-02-27)
------------------

1.7.4 (2017-02-27)
------------------

1.7.3 (2017-02-26)
------------------
* Stop lg_sv in the background
  Need the resources elsewhere.
* Contributors: Matt Vollrath

1.7.2 (2017-02-24)
------------------

1.7.1 (2017-02-23)
------------------

1.7.0 (2017-02-22)
------------------
* basic lg_sv doesn't use the inverted option
* Add lg_sv scene test script
* Fix lint in lg_sv webapps
* Add Google logo back to SV webapp
  Make it a more TOS-friendly presentation.
  Also revamp attribution styling to match.
* Consolidate lg_sv webapp libs
  Also upgrades lg_sv's THREE.js to R83.
* Reduce size of Street View link chevrons
* Embed libraries for Street View webapp
* Clean up redundant param in sv.launch
* Add link visualization to Street View webapp
* Disable pointer events in Street View webapp
  Prevent UI artifacts from stray input.
* Disable Street View default links UI entirely
  Not useful when oversizing the canvas.
* Use StreetViewService for attribution in webapp
* Clean up some console output in SV webapp
* Add basic Street View attribution module
  This uses an existing data stream to put the pano description on the screen.
  Needs better data for names, dates and copyrights.
* Implement tilt by default in Street View webapp
  Also remove some conflicting parameters.
* Cleanup in Street View webapp index
* Use Maps API 3.27 in Street View
* Disable throttling in sv webapp
* Fix whitespace in Street View viewer JS
* Contributors: Jacob Minshall, Matt Vollrath

1.6.5 (2017-02-08)
------------------

1.6.4 (2017-02-07)
------------------
* panovideosync: do not loop videos
* Some contained hacks for panovideosync
  In lieu of real backend, this will work for initial demonstration.
* Fix zero heading in pano video webapp
  Use the intended forward direction.
* Contributors: Matt Vollrath

1.6.3 (2017-02-03)
------------------
* Add synchronized panoramic video player
* Add street view dev launch file
* Contributors: Matt Vollrath

1.6.2 (2017-01-25)
------------------

1.6.1 (2017-01-12)
------------------

1.6.0 (2016-12-23)
------------------
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
* Contributors: Wojciech Ziniewicz

1.5.26 (2016-12-21)
-------------------

1.5.25 (2016-12-14)
-------------------

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

1.5.18 (2016-11-14)
-------------------

1.5.17 (2016-11-11)
-------------------

1.5.16 (2016-11-07)
-------------------

1.5.15 (2016-11-04)
-------------------

1.5.14 (2016-11-04)
-------------------

1.5.13 (2016-11-04)
-------------------

1.5.12 (2016-11-03)
-------------------

1.5.11 (2016-11-03)
-------------------

1.5.10 (2016-10-31)
-------------------

1.5.9 (2016-10-28)
------------------

1.5.8 (2016-10-27)
------------------

1.5.7 (2016-10-27)
------------------

1.5.6 (2016-10-26)
------------------

1.5.5 (2016-10-26)
------------------

1.5.4 (2016-10-25)
------------------

1.5.3 (2016-10-25)
------------------

1.5.2 (2016-10-19)
------------------

1.5.1 (2016-10-19)
------------------

1.5.0 (2016-10-19)
------------------

1.4.19 (2016-10-18)
-------------------

1.4.18 (2016-10-17)
-------------------

1.4.17 (2016-10-13)
-------------------

1.4.16 (2016-10-13)
-------------------

1.4.15 (2016-10-13)
-------------------

1.4.14 (2016-10-11)
-------------------

1.4.13 (2016-10-10)
-------------------

1.4.12 (2016-10-07)
-------------------

1.4.11 (2016-10-06)
-------------------

1.4.10 (2016-10-06)
-------------------

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
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz

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
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz

* Implement page urls monitor extension (`#293 <https://github.com/EndPointCorp/lg_ros_nodes/issues/293>`_)
  * Urls monitoring
  * Parse allowed urls config from get args
  * page monitor parameters passing
  * Page urls monitoring: readme, tests and get_args passing
  * Add allowed urls to adhoc browser message
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
* Contributors: Dmitry Kiselev, Wojciech Ziniewicz

1.4.6 (2016-09-28)
------------------

1.4.5 (2016-09-21)
------------------

1.4.4 (2016-09-21)
------------------

1.4.3 (2016-09-12)
------------------

1.4.2 (2016-09-12)
------------------

1.4.1 (2016-09-12)
------------------

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

1.3.30 (2016-08-31)
-------------------

1.3.29 (2016-08-31)
-------------------
* synced broken changelogs
* Contributors: Wojciech Ziniewicz

1.3.28 (2016-08-26)
-------------------

1.3.27 (2016-08-23)
-------------------

1.3.26 (2016-08-15)
-------------------
* lg_sv: kill chrome on soft relaunch
* Contributors: Jacob Minshall

1.3.25 (2016-08-12)
-------------------

1.3.24 (2016-08-12)
-------------------

1.3.23 (2016-08-09)
-------------------

1.3.22 (2016-08-09)
-------------------
* generating changelogs to satisfy jenkins lg_ros_nodes_deb_builds_master, touch: `#113 <https://github.com/EndPointCorp/lg_ros_nodes/issues/113>`_
* Contributors: Zdenek Maxa

1.3.21 (2016-08-03)
-------------------

1.3.20 (2016-07-29)
-------------------

1.3.19 (2016-07-29)
-------------------

1.3.18 (2016-07-28)
-------------------

1.3.17 (2016-07-27)
-------------------

1.3.16 (2016-07-26)
-------------------

1.3.15 (2016-07-26)
-------------------

1.3.14 (2016-07-25)
-------------------

1.3.13 (2016-07-21)
-------------------

1.3.12 (2016-07-19)
-------------------

1.3.11 (2016-07-15)
-------------------

1.3.10 (2016-07-13)
-------------------

1.3.9 (2016-07-08)
------------------

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

1.3.2 (2016-06-29)
------------------

1.3.1 (2016-06-28)
------------------
* fix timeout variable
* refactored x_available to DRY out code mode
* factor out dependency_available to check_www_dependency
  This DRYs out the code a bunch.
* Simplified street view nav snapping
  * Wait until nav is idle to snap back to horizontal.
* Contributors: Jacob Minshall, Matt Vollrath, Will Plaut

1.3.0 (2016-06-25)
------------------
* Introduce tilt snappiness
  * Use time series for smooth ephemeral tilt.
  * Keep old tilt behavior, settable at runtime with the tilt_snappy topic.
  * Slow down movement repeat.
  * Set constant zoom.
* Reduce street view nav gutter value
  Helps tilt snappiness work.
* Reduce Street View tick rate
* Fix `#230 <https://github.com/EndPointCorp/lg_ros_nodes/issues/230>`_ and add tests
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.2.14 (2016-06-10)
-------------------

1.2.13 (2016-06-10)
-------------------

1.2.12 (2016-06-07)
-------------------
* Ensure street view pov reset on transition
* Contributors: Matt Vollrath

1.2.11 (2016-06-02)
-------------------
* ignore spacenav messages when not visible in streetview
* Contributors: Jacob Minshall

1.2.10 (2016-05-20)
-------------------

1.2.9 (2016-05-20)
------------------

1.2.8 (2016-05-19)
------------------

1.2.7 (2016-05-17)
------------------

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
* Contributors: Wojciech Ziniewicz

1.2.1 (2016-05-03)
------------------
* Always send most recent Street View pov
  Prevent missing pov at webapp launch.
* Contributors: Matt Vollrath

1.2.0 (2016-04-29)
------------------
* lg_sv: Camera timer
  Passive SpaceNav message consumption.
* Contributors: Matt Vollrath

1.1.50 (2016-04-27)
-------------------
* move new loginfo logging to logdebug
* fix up logging
  Move some logerrs to log{warn,info} depending on the information being
  logged. Also s/logger.erroror/logger.error/
* Contributors: Jacob Minshall

1.1.49 (2016-04-26)
-------------------

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
* start listening on spacenav_wrapper/twist topic
* softrelaunch initial work
* Contributors: Jacob Minshall, Zdenek Maxa

* start listening on spacenav_wrapper/twist topic
* softrelaunch initial work
* Contributors: Jacob Minshall

1.1.41 (2016-04-13)
-------------------
* Generated changelogs while preparing for new release
* Contributors: Zdenek Maxa

1.1.40 (2016-03-23)
-------------------

1.1.39 (2016-03-16)
-------------------

1.1.38 (2016-03-09)
-------------------

1.1.37 (2016-03-04)
-------------------
* ignore buttons when state is false
* listen in on the proper metadata topic
* attribution card showing / hiding
* Contributors: Jacob Minshall

1.1.36 (2016-02-17)
-------------------
* add missing dependency
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
* panoviewer: unload meshes to reduce memory use
* lg_sv: default to boolean not string
  Plus explicit checking for the boolean true.
* Contributors: Jacob Minshall

1.1.30 (2016-01-11)
-------------------

1.1.29 (2016-01-04)
-------------------
* readme: updates to reflect params / topics
* Stop piling up messages in Chrome's debug log.
* Contributors: Adam Vollrath, Jacob Minshall

1.1.28 (2015-12-10)
-------------------
* lg_sv: use .get to access dicts to avoid key errors
* Contributors: Jacob Minshall

1.1.27 (2015-11-25)
-------------------
* lg_sv: add description and attribution_name to metadata
  Raw metadata from the client is trimmed down to ignore fields we don't
  use, but we are now using both of those fields.

1.1.26 (2015-11-25)
-------------------
* Merge pull request `#112 <https://github.com/EndPointCorp/lg_ros_nodes/issues/112>`_ from EndPointCorp/ft-change_panos_via_button
  lg_sv: move forward if a button has been clicked
* Revert "WIP on changing panos pointing to the closest link"
  This reverts commit a38e0e036faeb6192c412b6bb075eaf5e53766c0.
  More work needs to be put into this commit before merging it.
* WIP on changing panos pointing to the closest link
* lg_sv: different buttons do different things
* pep8 fix
* lg_sv: move forward if a button has been clicked
* Contributors: Adam Vollrath, Jacob Minshall

1.1.25 (2015-11-17)
-------------------
* Add titlecard to lg_sv panoviewer, fix fonts in lg_sv
* Contributors: Szymon Lipiński

1.1.24 (2015-11-16)
-------------------
* lg_sv: remove 42-b hard coding in favor of parameterized ros url
* lg_sv: resets zoom after changing panos
* lg_sv: some changes for specific lgs
* lg_sv: optional zoom for streetview
  This really only works well if you're only using one screen. Kind of
  counter productive to the heart of a liquid galaxy, but hey, at least
  it's kind of in there now.
* lg_sv: parameterized zoom level
* lg_sv: parameterization for rosbridge url
* lg_sv: set initial pano via url
  Using panoid=foobar will set the initial pano to point to foobar.
* lg_sv: reset tilt/heading if none are specified
  Zoom also always reverts to the default max zoom out
* Contributors: Jacob Minshall, Wojciech Ziniewicz

1.1.23 (2015-11-13)
-------------------
* Changed title for pano viewers
* Contributors: Wojciech Ziniewicz

1.1.22 (2015-11-05)
-------------------
* Add titlecard to the lg_sv sv viewer
* panoviewer: allow images from the headnode
* Contributors: Jacob Minshall, Szymon Lipiński

1.1.21 (2015-10-22)
-------------------
* lg_sv: invert the heading given by the director
* Contributors: Matt Vollrath

1.1.20 (2015-10-21)
-------------------
* init nearbypano parent class
* Contributors: Jacob Minshall

1.1.19 (2015-10-20)
-------------------
* lg_sv: allow for an inverted nearby pano finder
* lg_sv: changed default zoom\_{min,max}
* Contributors: Jacob Minshall

1.1.18 (2015-10-20)
-------------------
* lg_sv: server: fix nearby pano chooser
  This was returning an invalid difference for certain inputs.
* Contributors: Jacob Minshall

1.1.17 (2015-10-16)
-------------------
* lg_sv: invert the zoom value
* lg_sv: update zoom defaults
* lg_sv: zoom publishing
* lg_sv: handle null headers and tilt
* lg_sv: translate numbers to floats instead of strings
* lg_sv: raw metadata translation
* lg_sv: handles heading + tilt parameters in streetview asset
* Removed UBL
* Contributors: Jacob Minshall, Wojciech

1.1.16 (2015-10-11)
-------------------
* Added streetview client ROSbridge dependency
* Contributors: Wojciech Ziniewicz

1.1.15 (2015-10-10)
-------------------

1.1.14 (2015-10-08)
-------------------
* lv_sv: only split on / for a streetview pano
  Panoviewer panos are usually filenames
* lg_sv: handle panoids prepended by urls
* Contributors: Jacob Minshall

1.1.13 (2015-10-08)
-------------------

1.1.12 (2015-10-07)
-------------------
* lg_sv: parametrize the nearby pano class
* Contributors: Jacob Minshall, Wojciech Ziniewicz

1.1.11 (2015-10-06)
-------------------

1.1.10 (2015-10-05)
-------------------
* lg_sv: actually use the supplied x_threshold
* Added lots of docs

1.1.9 (2015-09-25)
------------------
* Dont start application if X is not available
* Better logging for dependencies
* ADded dependency checking and fixed slots deserialization
* panoviewer: replay videos that are republished
* lg_sv: parameterize tilt
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.1.8 (2015-09-25)
------------------
* lg_sv: parameterize x_threshold
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.1.7 (2015-09-24)
------------------
* PEP8
* Contributors: Adam Vollrath

1.1.6 (2015-09-24)
------------------
* Hide SV at startup
* Add queue_size to sv server state Publisher
* Contributors: Adam Vollrath, Matt Vollrath, Wojciech Ziniewicz

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
* lg_media: parameterized the videosync hardcoded values
* lg_media: browser adhoc player
  Launches videosync on any browser_media type messages from the director.
* lg\_{common,sv}: used the new director listener abstraction
* lg_sv: only set transform when shouldTilt is selected
  Plus jquery!
* lg_sv: parameterize tilt, default to false
* webapp: added videosync to webapps directory
  A slight change was made to parameterize the rosbridge url, and to use
  libraries from CDNs.
* lg_sv: use correct callback for director messages
* lg\_{sv,pv}: director message translation
* refactored panoviewer to unclog the global namespace
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.0.9 (2015-09-09)
------------------

1.0.8 (2015-08-12)
------------------

1.0.7 (2015-08-12)
------------------

1.0.6 (2015-08-10)
------------------

1.0.5 (2015-08-03)
------------------

1.0.4 (2015-07-31)
------------------
* JS lint cleanup and added JSDoc to sv_pov
* Tune spacenav handling
* Cleaned up client code and moved pov functions out
  -Also parameterized FOV into the launcher URL
* Contributors: Will Plaut

1.0.3 (2015-07-29)
------------------

1.0.2 (2015-07-29)
------------------

1.0.1 (2015-07-29)
------------------

0.0.7 (2015-07-28)
------------------
* Cleanup debugging output
* Fix movement and tune thresholds
* Use canvas/viewport ratio
  -Also increased canvas size
* Contributors: Will Plaut

0.0.6 (2015-07-28)
------------------
* Fix up lg_sv formatting for pep8
* Contributors: Will Plaut

0.0.5 (2015-07-27)
------------------
* Initial lg_sv package
* Contributors: Jacob Minshall, Kannan Ponnusamy, Matt Vollrath, Will Plaut
