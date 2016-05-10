^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_media
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
