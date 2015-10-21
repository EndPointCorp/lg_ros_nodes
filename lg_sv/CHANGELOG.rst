^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_sv
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

1.1.8 (2015-09-25)
------------------
* lg_sv: parameterize x_threshold
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

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
* Contributors: Wojciech

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
* Fixed 'depepency' typo
* ADded dependency checking and fixed slots deserialization
* panoviewer: replay videos that are republished
* 1.1.8
* catkin_generate_changelog
* pep8 fixes
* lg_sv: parameterize x_threshold
* Added ext dependency mechanism and added it to GE and SV/PV
* 1.1.7
* Small changes
* PEP8
* 1.1.6
* Updated changelogs
* Hide SV at startup
* A try on SV
* Add queue_size to sv server state Publisher
* 1.1.5
* Bumped changelgs
* 1.1.4
* lg_sv: parameterize tilt
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

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

0.0.4 (2015-07-27 15:11)
------------------------

0.0.3 (2015-07-21 18:14)
------------------------

0.0.2 (2015-07-21 17:11)
------------------------

0.0.1 (2015-07-08)
------------------
