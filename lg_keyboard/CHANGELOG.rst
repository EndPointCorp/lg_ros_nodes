^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_keyboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* fix route_touch_to_viewports
  No longer filter by activity_type.
* Contributors: Jacob Minshall

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
* tests implemented, fixed, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* PEP8
* in the middle of fixing onboard router online tests, not yet fixed, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* refactoring, implemented onboard_router offline tests, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* refactoring, removal of onboard manager, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Fix and refactor onboard classes
  One problem was the the ManagedApplication was given its window as a positional argument, so it set shell=(a ManagedWindow instance which is True) and did not get a window at all.
  Also, empty activation lists would not properly hide onboard.
* Onboard: always disable docking
* Onboard: move force-to-top to correct section
  Found it at http://bazaar.launchpad.net/~onboard/onboard/trunk/view/2181/Onboard/Config.py#L788
* Amended some stuff for `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* fixed hiding onboard keyboard, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Changed w_class for onboard `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Added onboard config gfor `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_ and made some back and forth on launching
* Fixed executor and started fixing test `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Onboard launcher `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
  - added OnboardLauncher executor
  - made methods for showing and hiding launcher
* onboard executor starter, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Forgot about state saving
* Added some idempotency and locking
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
* Added a stub of onboard router
* Fixed tests after renaming of the node to lg_keyboard
* Contributors: Matt Vollrath, Wojciech Ziniewicz, Zdenek Maxa

* Generated changelog
* tests implemented, fixed, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* PEP8
* in the middle of fixing onboard router online tests, not yet fixed, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* refactoring, implemented onboard_router offline tests, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* refactoring, removal of onboard manager, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Fix and refactor onboard classes
  One problem was the the ManagedApplication was given its window as a positional argument, so it set shell=(a ManagedWindow instance which is True) and did not get a window at all.
  Also, empty activation lists would not properly hide onboard.
* Onboard: always disable docking
* Onboard: move force-to-top to correct section
  Found it at http://bazaar.launchpad.net/~onboard/onboard/trunk/view/2181/Onboard/Config.py#L788
* Amended some stuff for `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* fixed hiding onboard keyboard, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Changed w_class for onboard `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Added onboard config gfor `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_ and made some back and forth on launching
* Fixed executor and started fixing test `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Onboard launcher `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
  - added OnboardLauncher executor
  - made methods for showing and hiding launcher
* onboard executor starter, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Forgot about state saving
* Added some idempotency and locking
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
* Added a stub of onboard router
* Fixed tests after renaming of the node to lg_keyboard
* Contributors: Matt Vollrath, Wojciech Ziniewicz, Zdenek Maxa

* tests implemented, fixed, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* PEP8
* in the middle of fixing onboard router online tests, not yet fixed, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* refactoring, implemented onboard_router offline tests, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* refactoring, removal of onboard manager, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Fix and refactor onboard classes
  One problem was the the ManagedApplication was given its window as a positional argument, so it set shell=(a ManagedWindow instance which is True) and did not get a window at all.
  Also, empty activation lists would not properly hide onboard.
* Onboard: always disable docking
* Onboard: move force-to-top to correct section
  Found it at http://bazaar.launchpad.net/~onboard/onboard/trunk/view/2181/Onboard/Config.py#L788
* Amended some stuff for `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* fixed hiding onboard keyboard, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Changed w_class for onboard `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Added onboard config gfor `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_ and made some back and forth on launching
* Fixed executor and started fixing test `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Onboard launcher `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
  - added OnboardLauncher executor
  - made methods for showing and hiding launcher
* onboard executor starter, touch: `#127 <https://github.com/EndPointCorp/lg_ros_nodes/issues/127>`_
* Forgot about state saving
* Added some idempotency and locking
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
* Added a stub of onboard router
* Fixed tests after renaming of the node to lg_keyboard
* Contributors: Matt Vollrath, Wojciech Ziniewicz, Zdenek Maxa

1.4.6 (2016-09-28)
------------------
