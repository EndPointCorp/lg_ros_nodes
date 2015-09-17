^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_sv
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
