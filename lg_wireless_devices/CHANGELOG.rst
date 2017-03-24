^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_wireless_devices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Made managed adhoc browser' tests' setUp and tearDown methods great a… (`#319 <https://github.com/endpointcorp/lg_ros_nodes/issues/319>`_)
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
* Topic/various ros nodes (`#277 <https://github.com/EndPointCorp/lg_ros_nodes/issues/277>`_)
  * Added spacenav_gmaps ros node
  * Added wireless devices and lg_rfreceiver nodes
  * Renamed spacenav_gmaps to lg_spacenav_globe
  * Fixed cmakelist for lg_spacenav_globe
  * Changed path for header files
  * Another header update
  * A bunch of name changes
  * Working out the deps
  * Added tests stub for proximity sensor
* Contributors: Wojciech Ziniewicz

1.4.0 (2016-09-09)
------------------

1.0.28 (2016-06-14)
-------------------

1.0.27 (2016-06-02)
-------------------
* pep8 fixes
* Contributors: Jacob Minshall

1.0.26 (2016-04-28)
-------------------
* update package.xml for wireless_devices
* Contributors: Jacob Minshall

1.0.25 (2016-04-08)
-------------------

1.0.24 (2016-04-08)
-------------------
* remove unused files
* wireless_devices: remove references to command_handler
* wireless_devices: move command handler outside this repo
* Use custom msg type: Command
* wireless_devices: move config files to custom directory
  Also start publishing on the original command_handler topic.
* wireless_watcher: move udev rule copying to wireless watcher
  Also since copying to /etc/udev needs sudo the udev rules are copied to
  a temporary file then os.system is used to copy them over with sudo.
  Plus the new udev rules are now triggered.
* Fix build and install
* Use our own Command message
* Added README for
* Split wireless handler and commands executor
* tmp commit for wireless remote on display nodes
* Contributors: Jacob Minshall, kiselev-dv
