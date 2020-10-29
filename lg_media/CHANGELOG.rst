^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_media
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2020-05-27)
------------------

Forthcoming
-----------

3.3.0 (2020-10-21)
------------------

3.2.9 (2020-09-21)
------------------

3.2.8 (2020-08-27)
------------------

3.2.7 (2020-08-25)
------------------

3.2.6 (2020-07-09)
------------------

3.2.5 (2020-07-06)
------------------

3.2.4 (2020-05-29)
------------------

3.2.3 (2020-05-28)
------------------

3.2.2 (2020-05-28)
------------------

3.2.1 (2020-05-27)
------------------
* 3.2.0
* BSPWM
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Galaxy Admin, Neil Elliott, dkiselev

3.1.12 (2020-05-04)
-------------------
* Fix media_launcher startup, shutdown issues
  multicast instead of unreliable RTSP server.
* Contributors: Matt Vollrath

3.1.11 (2020-05-01)
-------------------
* Fix PEP8
* Add basic media launcher (`#421 <https://github.com/EndPointCorp/lg_ros_nodes/issues/421>`_)
  Co-authored-by: Matt Vollrath <matt@endpoint.com>
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
* Contributors: Galaxy Admin, Matt Vollrath, Will Plaut, nelliott

3.1.10 (2020-03-25)
-------------------

3.1.9 (2020-03-11)
------------------

3.1.8 (2020-02-06)
------------------
* image_viewer: slight change for trasparency
* Contributors: Will Plaut

3.1.7 (2020-02-04)
------------------

3.1.6 (2020-01-27)
------------------
* image_viewer: initial scene functionality
* Contributors: Jacob Minshall

3.1.5 (2020-01-24)
------------------
* decode in the right place
* Contributors: Jacob Minshall

3.1.4 (2020-01-24)
------------------
* Merge pull request `#414 <https://github.com/EndPointCorp/lg_ros_nodes/issues/414>`_ from EndPointCorp/topic/image_viewer_fixes_and_tracebacks
  Topic/image viewer fixes and tracebacks
* image_checker: fix encoding issue
* image_viewer: remove debuggin
* changes need to be cleaned up
* Contributors: Jacob Minshall, Will Plaut

3.1.3 (2020-01-20)
------------------

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
* starting to update CMakes and package.xmls
* Finish pycodestyle fixing image_viewer
* Fix pointer glitches in gst_video_sync
* Fix gst_video_sync style glitches
* Fix some pycodestyle glitches
* update import paths everywhere
* Contributors: Matt Vollrath, Will Plaut

3.0.2 (2019-11-06)
------------------

3.0.1 (2019-11-06)
------------------
* image_checker: python3 shebang
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes
* Contributors: Will Plaut, dkiselev

3.0.0 (2019-10-31)
------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/python_tree
  Conflicts:
  rosbridge_library/CHANGELOG.rst
  rosbridge_library/package.xml
  rosbridge_server/CHANGELOG.rst
  rosbridge_server/package.xml
* Merge pull request `#407 <https://github.com/EndPointCorp/lg_ros_nodes/issues/407>`_ from EndPointCorp/fix_gst_video_sync_mp3_streaming
  Fix gst_video_sync MP3 streaming
* gst_video_sync: Keep trying duration query
  When streaming MP3 over HTTP sometimes it can't initially query the duration.
  Keep trying the query, making sure to cancel when we quit.
* gst_video_sync: Ensure quit() exits
* set python executable for tests
* 2to3 all of it
* python 2 shebang to 3
* Contributors: Matt Vollrath, Will Plaut

2.0.18 (2019-10-11)
-------------------
* Merge pull request `#406 <https://github.com/EndPointCorp/lg_ros_nodes/issues/406>`_ from EndPointCorp/topic/image_checker
  image_checker: crosscheck uscs msgs with current procs
* image_checker: use threading timer
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into topic/image_checker
* image_checker: remove debuging stuff
  also adds rosparam for sleep timeout
* image_checker: fixup naming and such
* image_checker: small changes
* image_checker: use uscs to verify
* image_checker: first round of changes
* image_checker: cleanup
* image_checker: working bones
* image_checker: fix syntax errors
* lg_media: add script to CMake
* image_checker: crosscheck uscs msgs with current procs
* Contributors: Will Plaut

2.0.17 (2019-09-11)
-------------------

2.0.16 (2019-09-06)
-------------------
* gst_video_sync: disable aggressive downloading
  Don't fill up the disk with this cruft.
* Contributors: Matt Vollrath

2.0.15 (2019-08-20)
-------------------
* Manage libqt5gstreamer build dep properly
  This was merged into rosdistro.
* gst_video_sync: Don't mess with buffer size
  The small buffer was causing audio issues for some videos.
* Contributors: Matt Vollrath

2.0.14 (2019-08-19)
-------------------
* Disable hardware decoding for gst_video_sync
  Known to be broken when combining vaapi<->vdpau
* gst_video_sync: Check state changes
* gst_video_sync: Use autovideosink
  Disabling hardware decoding was not working because we specified vaapisink.
* Contributors: Jacob Minshall, Matt Vollrath

2.0.13 (2019-07-29)
-------------------
* gst_video_sync: Args for window geometry
  Awesome WM was having trouble setting x and y, so do it in the app.
* Contributors: Matt Vollrath

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

2.0.7 (2019-07-03)
------------------
* gst_video_sync: specify buffer size
* Normalize roslib topic throttle and queues
* Contributors: Matt Vollrath

2.0.6 (2019-07-02)
------------------
* gst_video_sync: slaves won't wait for master
* gst_video_sync: Hide init, use vaapi and vdpau
* Contributors: Matt Vollrath

2.0.5 (2019-07-02)
------------------
* gst_video_sync: use xvimagesink
* Contributors: Matt Vollrath

2.0.4 (2019-07-02)
------------------
* Add lg_media gstreamer_pool
  For launching gst_video_sync.
  A mostly drop-in replacement for mplayer_pool.
  Custom mplayer args in scenes will need to be ported.
* GStreamer-based sync video player
* Update browser video app
  Stripped down version of the lg_panovideo player with optional sync.
* Contributors: Matt Vollrath

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
* Merge pull request `#398 <https://github.com/EndPointCorp/lg_ros_nodes/issues/398>`_ from EndPointCorp/topic/image_viewer_two
  image_viewer: pqiv and feh
* image_viewer: better loggin levels
* image_viewer: pqiv and feh
  Our powers combined!
* Contributors: Will Plaut

1.20.2 (2019-05-22)
-------------------

1.20.1 (2019-05-21)
-------------------

1.20.0 (2019-05-15)
-------------------
* PEP8 sweep
* lg_media: fix package.xml
* Add image viewer node with transparent image display
* Contributors: Matt Vollrath, Will Plaut

1.19.16 (2019-05-14)
--------------------
* Merge pull request `#396 <https://github.com/EndPointCorp/lg_ros_nodes/issues/396>`_ from EndPointCorp/topic/transparent_images
  Topic/transparent images
* lg_media: fix package.xml
* image_viewer: cleanup
* image_viewer: simplified a bit
* image_viewer: syntax
* image_viewer: moved to pqiv and cleanup
* image_viewer: fix image comparison
* more working image_viewer.py
* mostly working image viewer
* Contributors: Will Plaut

1.19.15 (2019-04-29)
--------------------

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
* Fix Change: re-add line
* VIDEO-SYNC Ros node changes
* Contributors: Galaxy Admin

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

1.17.4 (2017-11-10)
-------------------

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

1.14.1 (2017-07-17)
-------------------

1.14.0 (2017-07-14)
-------------------

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

1.11.3 (2017-03-31)
-------------------

1.11.2 (2017-03-31)
-------------------

1.11.1 (2017-03-28)
-------------------

1.11.0 (2017-03-27)
-------------------

1.10.2 (2017-03-24)
-------------------

1.10.1 (2017-03-23)
-------------------

1.10.0 (2017-03-23)
-------------------

1.9.1 (2017-03-20)
------------------

1.9.0 (2017-03-20)
------------------

1.8.0 (2017-03-09)
------------------

1.7.11 (2017-03-03)
-------------------

1.7.10 (2017-03-02)
-------------------

1.7.9 (2017-03-01)
------------------

1.7.8 (2017-03-01)
------------------

1.7.7 (2017-02-28)
------------------

1.7.6 (2017-02-27)
------------------

1.7.5 (2017-02-27)
------------------

1.7.4 (2017-02-27)
------------------

1.7.3 (2017-02-26)
------------------

1.7.2 (2017-02-24)
------------------

1.7.1 (2017-02-23)
------------------

1.7.0 (2017-02-22)
------------------

1.6.5 (2017-02-08)
------------------

1.6.4 (2017-02-07)
------------------

1.6.3 (2017-02-03)
------------------

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
* Contributors: Wojciech Ziniewicz

* Generated changelog
* Contributors: Wojciech Ziniewicz

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

1.3.28 (2016-08-23)
-------------------

1.3.27 (2016-08-23)
-------------------

1.3.26 (2016-08-15)
-------------------
* add soft relaunch support for all media assets
  browser graphics, browser videos, and mplayer videos supported.
* Contributors: Jacob Minshall

1.3.25 (2016-08-12)
-------------------

1.3.24 (2016-08-12)
-------------------

1.3.23 (2016-08-09)
-------------------

1.3.22 (2016-08-09)
-------------------

1.3.21 (2016-08-03)
-------------------

1.3.20 (2016-07-29)
-------------------
* Fix MPlayer looping
  The -loop needs to come after the url.  Obviously!
  Also, go back to specifying full geometry because it works.
* Contributors: Matt Vollrath

1.3.19 (2016-07-29)
-------------------
* Normalize MPlayer window positioning
* Contributors: Matt Vollrath

1.3.18 (2016-07-28)
-------------------

1.3.17 (2016-07-27)
-------------------

1.3.16 (2016-07-26)
-------------------

1.3.15 (2016-07-26)
-------------------
* Use mplayer -loop arg when appropriate
* Clear mplayer FIFO's on shutdown
* Mplayer pool asset persistence across scenes
* Lock mplayer pool public methods
  Prevent race conditions in message handlers.
* Remove shebang from mplayer_pool module
* Adjust mplayer default args
  * Prefer ipv4 to prevent observed ipv6 lookup errors
  * Increase cache size to support high bitrate video streaming
* Contributors: Matt Vollrath

1.3.14 (2016-07-25)
-------------------

1.3.13 (2016-07-21)
-------------------

1.3.12 (2016-07-19)
-------------------

1.3.11 (2016-07-15)
-------------------
* Merge branch 'master' of github.com:EndPointCorp/lg_ros_nodes into EndPointCorp/lg_chef_860_graphics_loading_timing_issues
  Conflicts:
  lg_common/src/lg_common/adhoc_browser_pool.py
  lg_media/src/lg_media/mplayer_pool.py
* Removed instance updating completely
* Contributors: Galaxy Admin

1.3.10 (2016-07-13)
-------------------
* Fixed `EndPointCorp/lg_chef#859 <https://github.com/EndPointCorp/lg_chef/issues/859>`_
* Fixed browser vid player
* Contributors: Galaxy Admin, Wojciech Ziniewicz

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

1.3.0 (2016-06-25)
------------------

1.2.14 (2016-06-10)
-------------------
* fixed tests, touch: `#193 <https://github.com/EndPointCorp/lg_ros_nodes/issues/193>`_
* Contributors: Zdenek Maxa

1.2.13 (2016-06-10)
-------------------
* mplayer control behaviour, looping videos, `#193 <https://github.com/EndPointCorp/lg_ros_nodes/issues/193>`_
* Contributors: Zdenek Maxa

1.2.12 (2016-06-07)
-------------------

1.2.11 (2016-06-02)
-------------------

1.2.10 (2016-05-20)
-------------------

1.2.9 (2016-05-20)
------------------

1.2.8 (2016-05-19)
------------------
* remove write_log_to_file imports
* removed mockity mock, mplayer tests exceluded from jenkins runs, touch: `#215 <https://github.com/endpointcorp/lg_ros_nodes/issues/215>`_
* Contributors: Jacob Minshall, Zdenek Maxa

1.2.7 (2016-05-17)
------------------

1.2.6 (2016-05-16)
------------------

1.2.5 (2016-05-12)
------------------

1.2.4 (2016-05-10)
------------------

1.2.3 (2016-05-06)
------------------
* Generated changelogs
* 1.2.2
* Contributors: Wojciech Ziniewicz

1.2.1 (2016-05-03)
------------------
* Cleanup some unneeded testing nodes
* Contributors: Matt Vollrath

1.2.0 (2016-04-29)
------------------

1.1.50 (2016-04-27)
-------------------
* move new loginfo logging to logdebug
* fix up logging
  Move some logerrs to log{warn,info} depending on the information being
  logged. Also s/rospy.logerror/rospy.logerr/
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

1.1.41 (2016-04-14)
-------------------

1.1.40 (2016-03-23)
-------------------

1.1.39 (2016-03-16)
-------------------

1.1.38 (2016-03-09)
-------------------
* Fixed unittest
* Mockity mock for mplayer tests
* Disabled mplayer real tests as we're not using them
* Contributors: Wojciech Ziniewicz

1.1.37 (2016-03-04)
-------------------

1.1.36 (2016-02-17)
-------------------

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

1.1.30 (2016-01-11)
-------------------

1.1.29 (2016-01-04)
-------------------
* lg_media: specify python-pytest dependency
* Contributors: Jacob Minshall

1.1.28 (2015-12-10)
-------------------

1.1.27 (2015-11-25)
-------------------

1.1.26 (2015-11-25)
-------------------

1.1.25 (2015-11-17)
-------------------

1.1.24 (2015-11-16)
-------------------
* Add mplayer to lg_media run_depends
* increased mplayer timeout
* Contributors: Matt Vollrath, Wojciech Ziniewicz, Zdenek Maxa

1.1.23 (2015-11-13)
-------------------

1.1.22 (2015-11-05)
-------------------

1.1.21 (2015-10-22)
-------------------

1.1.20 (2015-10-21)
-------------------

1.1.19 (2015-10-20)
-------------------

1.1.18 (2015-10-20)
-------------------

1.1.17 (2015-10-16)
-------------------

1.1.16 (2015-10-11)
-------------------

1.1.15 (2015-10-10)
-------------------

1.1.14 (2015-10-08)
-------------------
* Remove UBL
* Contributors: Adam Vollrath

1.1.13 (2015-10-08)
-------------------
* Generate nice viewport slug
* Contributors: Adam Vollrath

1.1.12 (2015-10-07)
-------------------

1.1.11 (2015-10-06)
-------------------

1.1.10 (2015-10-05)
-------------------
* Added lots of docs
* mplayer occassional issues investigated and understood, should be fine now, touch:`#31 <https://github.com/endpointcorp/lg_ros_nodes/issues/31>`_

1.1.9 (2015-09-25)
------------------
* pep8 fixes
* Contributors: Adam Vollrath, Jacob Minshall, Zdenek Maxa

1.1.8 (2015-09-25)
------------------
* added real mplayer test scenarios, still needs debugging
* Contributors: Adam Vollrath, Jacob Minshall, Zdenek Maxa

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
* Added mplayer todo
* Contributors: Wojciech Ziniewicz

1.1.2 (2015-09-22)
------------------

1.1.1 (2015-09-18)
------------------

1.1.0 (2015-09-17)
------------------
* lg_media: parameterized the videosync hardcoded values
* lg_media: browser adhoc player
  Launches videosync on any browser_media type messages from the director.
* lg_media: parameterize media_type in director_media_bridge
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.0.9 (2015-09-09)
------------------

1.0.8 (2015-08-12)
------------------

1.0.7 (2015-08-12)
------------------

1.0.6 (2015-08-10)
------------------
* lg_media: add non-default port to rosbridge
  Tests on the jenkins machine were interfering with eachother because
  port 9090 was in use.
* test module refactoring, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* tests cases coverage done (without geometry), touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* test files clean up, continue implementing ..., touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* rostest, rosunit, pytest experiments, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* py.test, rostest, nosetests experiments, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* rostest plus py.test works, incl. correct reporting, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* rostest plus py.test test class foundation, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* wrong way of handling roslaunch, media services test started, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* Contributors: Jacob Minshall, Zdenek Maxa

1.0.5 (2015-08-03)
------------------

1.0.4 (2015-07-31)
------------------
* service call, return info on tracked apps, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* fifo writing test, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* URL updates via FIFO file commands, touch: `#31 <https://github.com/EndPointCorp/lg_ros_nodes/issues/31>`_
* http URL video playing
* Contributors: Zdenek Maxa

1.0.3 (2015-07-29)
------------------

1.0.2 (2015-07-29)
------------------
* Initial release
* Contributors: Matt Vollrath, Wojciech Ziniewicz, Zdenek Maxa
