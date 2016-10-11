^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_spacenav_globe
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.4.13 (2016-10-10)
-------------------

1.4.12 (2016-10-07)
-------------------

1.4.11 (2016-10-06)
-------------------
* Changes t spacenav globe and ros window ready extension
* Contributors: wojciech ziniewicz

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
* Fixed spacenav globe executable creation
* Contributors: Wojciech Ziniewicz

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
-------------------

1.0.28 (2016-06-14)
-------------------

1.0.27 (2016-06-02)
-------------------

1.0.26 (2016-04-28)
-------------------

1.0.25 (2016-04-08)
-------------------

1.0.24 (2016-04-08)
-------------------

1.0.23 (2016-04-06)
-------------------

1.0.22 (2016-04-06)
-------------------

1.0.21 (2016-04-06)
-------------------

1.0.20 (2016-02-03)
-------------------

1.0.19 (2016-02-01)
-------------------

1.0.18 (2016-02-01)
-------------------

1.0.17 (2016-01-18)
-------------------
* 1.0.16
* changelog bump
  Just changes to pano_app in here. No more awkward zoom when changing
  between panos in the runway. We track the current pov now instead of
  setting a random zoom.
* 1.0.15
* Changelogs for 1.0.15
* 1.0.14
* Changelogs for 1.0.14
* 1.0.13
* Changelogs for 1.0.13
* 1.0.12
* Changelogs
* 1.0.11
* Changelogs for 1.0.11
* 1.0.10
* maybe release notes should go here..
  Instead of just saying "bumped changelogs" I guess some good information
  should go here about why a release is being made... This is really just
  a small change to start setting the state so we can support videos with
  our pano app.
* 1.0.9
* bump changelogs
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.0.12 (2015-11-26)
-------------------

1.0.11 (2015-11-24)
-------------------

1.0.16 (2015-12-17)
-------------------
* 1.0.15
* Changelogs for 1.0.15
* 1.0.14
* Changelogs for 1.0.14
* 1.0.13
* Changelogs for 1.0.13
* 1.0.12
* Changelogs
* 1.0.11
* Changelogs for 1.0.11
* Contributors: Matt Vollrath, Wojciech Ziniewicz

1.0.10 (2015-11-20)
-------------------

1.0.9 (2015-11-19)
------------------

1.0.8 (2015-11-19)
------------------

1.0.7 (2015-11-17)
------------------

1.0.6 (2015-11-17)
------------------

1.0.5 (2015-11-16)
------------------

1.0.4 (2015-11-16)
------------------

1.0.3 (2015-11-16)
------------------
* Added proper changelog versions
* Contributors: Wojciech Ziniewicz

1.0.2 (2015-11-16)
------------------

1.0.0 (2015-11-13)
------------------

0.0.9 (2015-11-13)
------------------
* Downgraded package version temporarily before release
* Catkin release management
  - remove debian metadata that's duplicating catkin metadata
  - removed changelos for later autogeneration
  - edited all packages.xmls everywhere to reset to version 1.0
* unifying version
* initial changelog creation
* add urls to package.xml for lg_builder's sake
* LINT and cleanup for spacenav_gmaps package
* spring cleaning, updated all version numbers
* Bump spacenav_gmaps
* Tweak spacenav_gmaps queue lengths
* Bumped versions for new release
* Added support for replacing packages
* More info in spacenav_gmaps changelog
* Reduce default nav sensitivity, add param
  Can now set sensitivity with joystick_sensitivity param.
* Increment spacenav_gmaps version
* Fix SpaceNav polar crossing
  Flip longitude to actually traverse the pole instead of bouncing off of it.
* Use fabs() in spacenav_gmaps polar logic
* Update README for spacenav_gmaps package
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz
