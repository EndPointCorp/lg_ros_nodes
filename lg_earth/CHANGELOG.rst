^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_earth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.20 (2015-10-21)
-------------------

1.1.35 (2016-02-05)
-------------------
* lg_earth: curl custom config
* Contributors: Jacob Minshall

1.1.34 (2016-02-05)
-------------------
* lg_earth: make directory for localdbrootproto
* Contributors: Jacob Minshall

1.1.33 (2016-02-04)
-------------------
* lg_earth: copy local localdbrootproto
  This file needs contents to be useful, so grab it from the local filesystem.
* Set LANG from within lg_earth node
* Support custom earth configs in lg_earth node
* Contributors: Galaxy Admin, Jacob Minshall, Will Plaut

1.1.32 (2016-01-28)
-------------------
* Amended logging for `#137 <https://github.com/EndPointCorp/lg_ros_nodes/issues/137>`_
* Contributors: Wojciech Ziniewicz

1.1.31 (2016-01-20)
-------------------

1.1.30 (2016-01-11)
-------------------

1.1.29 (2016-01-04)
-------------------

1.1.28 (2015-12-10)
-------------------
* Turned ON {city,state}_{borders_labels} for default
* Contributors: Bryan Berry

1.1.27 (2015-11-25)
-------------------
* 1.1.26
* Changelogs
* Contributors: Wojciech Ziniewicz

1.1.25 (2015-11-17)
-------------------

1.1.26 (2015-11-25)
-------------------
* 1.1.25
* Generated new changelog
* Contributors: Szymon Lipiński

1.1.24 (2015-11-16)
-------------------
* 1.1.23
* Generated changelogs
* Contributors: Wojciech Ziniewicz

1.1.23 (2015-11-13)
-------------------

1.1.22 (2015-11-05)
-------------------

1.1.21 (2015-10-22)
-------------------
* 1.1.20
* Changelogs for 1.1.20
* Contributors: Matt Vollrath

1.1.19 (2015-10-20)
-------------------

1.1.18 (2015-10-20)
-------------------

1.1.17 (2015-10-16)
-------------------
* lg_earth: viewsync: update readme
* Contributors: Jacob Minshall

1.1.16 (2015-10-11)
-------------------

1.1.15 (2015-10-10)
-------------------

1.1.14 (2015-10-08)
-------------------

1.1.13 (2015-10-08)
-------------------

1.1.12 (2015-10-07)
-------------------
* kmlsync: escape this asset because it's unescaped later
  Ampersands will no longer cause this script to choke.
* Contributors: Jacob Minshall

1.1.11 (2015-10-06)
-------------------

1.1.10 (2015-10-05)
-------------------
* Added lots of docs
* Documentation
  - moved earth docs to lg_earth
  - added lg image

1.1.9 (2015-09-25)
------------------
* Added X dependency to Earth
* Dont start application if X is not available
* Better logging for dependencies
* ADded dependency checking and fixed slots deserialization
* Added ext dependency mechanism and added it to GE and SV/PV
* Mark kmlsync timeout experimental, default off
* Revisit KmlUpdateHandler.get()
* Remove crufty assignment from KmlUpdateHandler
* Improve KmlQueryHandler error messages
* No timeout for non-polling kmlsync test
* Improve KmlUpdateHandler deferral
* Improve kml create/delete logic and indentation
* Fix up KmlQueryHandler logic
* Further flatten KmlUpdateHandler.get()
* Refine KmlUpdateHandler timeout logic
  Lock all dict access, nothing else.
* Refactor KmlUpdateHandler asset change list funcs
* Whitespace, logging changes in KmlUpdateHandler
* Un-nest KmlUpdateHandler missing slug logic
* Rename KmlUpdateHandler global_dict
  Not a good identifier.
* Prevent race condition from happening in kmlsync
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.1.8 (2015-09-25)
------------------
* Added ext dependency mechanism and added it to GE and SV/PV
* Contributors: Adam Vollrath, Matt Vollrath, Wojciech Ziniewicz

1.1.7 (2015-09-24)
------------------

1.1.6 (2015-09-24)
------------------
* Mark kmlsync timeout experimental, default off
* Refactor KmlUpdateHandler asset change list funcs
* Contributors: Matt Vollrath

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
* Fix 1.0.9 changelogs
* Contributors: Jacob Minshall, Matt Vollrath

1.0.9 (2015-09-09)
------------------

1.0.8 (2015-08-12)
------------------

1.0.7 (2015-08-12)
------------------

1.0.6 (2015-08-10)
------------------
* Remove faulty KML unload
* Contributors: Will Plaut, Zdenek Maxa

1.0.5 (2015-08-03)
------------------

1.0.4 (2015-07-31)
------------------

1.0.3 (2015-07-29)
------------------

1.0.2 (2015-07-29)
------------------

1.0.1 (2015-07-29)
------------------

0.0.7 (2015-07-28)
------------------

0.0.6 (2015-07-28)
------------------
* Fix some catkin_lint issues
* Contributors: Matt Vollrath

0.0.5 (2015-07-27)
------------------

0.0.4 (2015-07-27)
------------------
* kmlsync: send playtourqueryrequest object instead of string
* kmlsync: move flyto unloading to state changes
* unload any flytos in progress on networkling_update & when searching
* Contributors: Jacob Minshall

0.0.3 (2015-07-21)
------------------
* Document changes to Earth client params
* Increase default Earth memory cache size
* Don't manage Earth window if gui is hidden
* Contributors: Matt Vollrath

0.0.2 (2015-07-21)
------------------
* Reduce default disk cache size for Earth client
* Optimize service requests in kmlsync
* Use persistent service proxies in kmlsync
* Allow unset viewport in lg_earth::client
* client_config: more google earth config lines
* remove google "Happy Birthday Earth" splash page
* kmlsync: unload assets even if no earth activity is supplied
* Contributors: Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

0.0.1 (2015-07-08)
------------------
* Initial release
* Contributors: Jacob Minshall, Kannan, Kannan Ponnusamy, Matt Vollrath, Wojciech Ziniewicz
