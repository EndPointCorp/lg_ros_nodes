^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_wireless_devices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
