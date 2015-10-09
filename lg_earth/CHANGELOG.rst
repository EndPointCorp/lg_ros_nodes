^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_earth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.7 (2015-09-24)
------------------

1.1.6 (2015-09-24)
------------------
* Mark kmlsync timeout experimental, default off
* Refactor KmlUpdateHandler asset change list funcs
* Contributors: Matt Vollrath

1.1.5 (2015-09-23)
------------------

1.1.8 (2015-09-25)
------------------
* Added ext dependency mechanism and added it to GE and SV/PV
* Contributors: Adam Vollrath, Matt Vollrath, Wojciech Ziniewicz

Forthcoming
-----------

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
* 1.1.8
* catkin_generate_changelog
* Added ext dependency mechanism and added it to GE and SV/PV
* 1.1.7
* Small changes
* 1.1.6
* Updated changelogs
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
* 1.1.5
* Bumped changelgs
* 1.1.4
* Prevent race condition from happening in kmlsync
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

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
