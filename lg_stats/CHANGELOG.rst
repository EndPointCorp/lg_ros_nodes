^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_stats
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Generated changelogs
* 1.2.2
* Added count_nonzero strategy for `#208 <https://github.com/EndPointCorp/lg_ros_nodes/issues/208>`_
* Contributors: Wojciech Ziniewicz

1.2.1 (2016-05-03)
------------------
* Disable tests for lg_stats
  Missing influxdb libraries.
  The builds must go on.
* Contributors: Matt Vollrath

1.2.0 (2016-04-29)
------------------

1.1.50 (2016-04-27)
-------------------
* fix up logging
  Move some logerrs to log{warn,info} depending on the information being
  logged. Also s/rospy.logerror/rospy.logerr/
* Contributors: Jacob Minshall

1.1.49 (2016-04-26)
-------------------
* PEP8
* Fixed a typo
* Lowered verbosity of lg_stats
* Fixed tests:
  - renamed files to reflect new functionality (new strategies) - tests
  coverage is missing for non-default ones
  - amended some code I wrongly added to meat
  - pep8'ized code
* Fixed tests for `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* Merge branch 'development' of github.com:EndPointCorp/lg_ros_nodes into development
* If value is float - submit it as float `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* lg_stats part 2
  - re-thinked activity sources parsing - lg_activity tests need to be
  written to make sure its not broken
  - moved count and average processors to background tasks like
  resubmitters - good idea by @zdenekmaxa
  - added `measurement` message field and moved attribute mapping so that
  ROS topic are independent from measurment names
* lg_stats strategies and activity sources:
  - added support for nested slots value extraction
  - refactored lg_activity to use shared helper for the above
  - removed cruft for strategies - replaced with proper strategies
  - added support for count and average
  - didnt test it yet - havent amended tests to resemble new functionality
  yet
* Minor formatting hanges
* Contributors: Wojciech Ziniewicz

1.1.48 (2016-04-20)
-------------------
* influxdb dependency, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* Contributors: Zdenek Maxa

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

1.1.43 (2016-04-14 14:29)
-------------------------

1.1.42 (2016-04-14 14:12)
-------------------------

1.1.41 (2016-04-13)
-------------------
* Fixed version of lg_Stats
* fixed lg_stats resubmission bug 2, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* fixed lg_stats resubmission bug, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* fix quotes in the test, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* fixing quotes, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* minor, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* added timestamps to influx messages, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_,`#181 <https://github.com/EndPointCorp/lg_ros_nodes/issues/181>`_
* minor, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* minor, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* thread worker rewritten to be offline testeable, online, offline tests separated, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* resubmision thread, so far w/o tests, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* covered source ros topics reviewed, satisfied lint, added real director scene msg test, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* bunch of other test cases added (slots, empty message, etc), touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* testing submitters, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* mock submitter for the tests, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* submit every message, non-empty messages checks (incl. slots), touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* started behaviour changes, tests fixed, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* implemented subslot, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* introduced message slot 2, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* introduced message slot, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* debugging influx submission condition, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* external dependency masked, debug statements added, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* telegraf submission via socket, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* refactored direct client influxdb connection, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* satisfying pep8, no default influxdb submission from tests, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* submission into influxdb implemented, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
  -using library recommended on influxdb.com
  -tests adjusted accoringly - two versions of ros test/roslaunch file with
  influxdb instance and without it - need to later find out how to
  mock influxdb better
  -removed hostname - will be part of static telegraf attributes
  -merely first pass of the InfluxDB tags - will be a subject of later evolution
* added basic, mocked, Processor class unittests, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* time resolution period test implemented, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* current tests refactored, code reused, shortened, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* added json string field, output message renamed, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* implemented time resolution and delayed message processing, Processor refactoring touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* more complex stats output message, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* source topic /activity/active handled, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* other topics handled, tests added, before tests refactoring now, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* dynamic source configuration done, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* first message listener - reaction done, tests working reliably, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* debugging occasional failure due to topic message not delivered, still in vain, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
* stats, /director/scene topic handled, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
  -checked against /lg_stats/debug topic
* initial work on `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_, touch: `#126 <https://github.com/EndPointCorp/lg_ros_nodes/issues/126>`_
  -ros nodes implementation files skeleton
  -testing aux files
  -compiles, test runs fine individually as well as within the test suite
* Contributors: Zdenek Maxa

1.1.40 (2016-03-23)
-------------------

1.1.39 (2016-03-16)
-------------------

1.1.38 (2016-03-09)
-------------------

1.1.37 (2016-03-04)
-------------------

1.1.36 (2016-02-17)
-------------------

1.1.35 (2016-02-05 12:02)
-------------------------

1.1.34 (2016-02-05 09:57)
-------------------------

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

1.1.28 (2015-12-10)
-------------------

1.1.27 (2015-11-25 11:44)
-------------------------

1.1.26 (2015-11-25 11:20)
-------------------------

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

1.1.19 (2015-10-20 21:30)
-------------------------

1.1.18 (2015-10-20 13:40)
-------------------------

1.1.17 (2015-10-16)
-------------------

1.1.16 (2015-10-11)
-------------------

1.1.15 (2015-10-10)
-------------------

1.1.14 (2015-10-08 17:02)
-------------------------

1.1.13 (2015-10-08 14:35)
-------------------------

1.1.12 (2015-10-07)
-------------------

1.1.11 (2015-10-06)
-------------------

1.1.10 (2015-10-05)
-------------------

1.1.9 (2015-09-25 20:51)
------------------------

1.1.8 (2015-09-25 09:13)
------------------------

1.1.7 (2015-09-24 13:57)
------------------------

1.1.6 (2015-09-24 02:12)
------------------------

1.1.5 (2015-09-23 21:09)
------------------------

1.1.4 (2015-09-23 20:33)
------------------------

1.1.3 (2015-09-22 14:18)
------------------------

1.1.2 (2015-09-22 12:01)
------------------------

1.1.1 (2015-09-18)
------------------

1.1.0 (2015-09-17)
------------------

1.0.9 (2015-09-09)
------------------

1.0.8 (2015-08-12 18:01)
------------------------

1.0.7 (2015-08-12 14:05)
------------------------

1.0.5 (2015-08-03)
------------------

1.0.4 (2015-07-31)
------------------

1.0.3 (2015-07-29 19:30)
------------------------

1.0.2 (2015-07-29 13:05)
------------------------

1.0.1 (2015-07-29 08:17)
------------------------

0.0.7 (2015-07-28 19:11)
------------------------

0.0.6 (2015-07-28 18:46)
------------------------

0.0.5 (2015-07-27 18:58)
------------------------

0.0.4 (2015-07-27 15:11)
------------------------

0.0.3 (2015-07-21 18:14)
------------------------

0.0.2 (2015-07-21 17:11)
------------------------
