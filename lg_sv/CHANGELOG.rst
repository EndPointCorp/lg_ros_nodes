^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_sv
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.20 (2015-10-21)
-------------------
* init nearbypano parent class
* Contributors: Jacob Minshall

1.1.29 (2016-01-04)
-------------------
* readme: updates to reflect params / topics
* Stop piling up messages in Chrome's debug log.
* Contributors: Adam Vollrath, Jacob Minshall

1.1.28 (2015-12-10)
-------------------
* lg_sv: use .get to access dicts to avoid key errors
* Contributors: Jacob Minshall

1.1.27 (2015-11-25)
-------------------
* lg_sv: add description and attribution_name to metadata
  Raw metadata from the client is trimmed down to ignore fields we don't
  use, but we are now using both of those fields.
* 1.1.26
* Changelogs
* Merge pull request `#112 <https://github.com/endpointcorp/lg_ros_nodes/issues/112>`_ from EndPointCorp/ft-change_panos_via_button
  lg_sv: move forward if a button has been clicked
* Revert "WIP on changing panos pointing to the closest link"
  This reverts commit a38e0e036faeb6192c412b6bb075eaf5e53766c0.
  More work needs to be put into this commit before merging it.
* WIP on changing panos pointing to the closest link
* lg_sv: different buttons do different things
* pep8 fix
* lg_sv: move forward if a button has been clicked
* Contributors: Adam Vollrath, Jacob Minshall, Wojciech Ziniewicz

1.1.25 (2015-11-17)
-------------------
* Add titlecard to lg_sv panoviewer, fix fonts in lg_sv
* Contributors: Szymon Lipiński

1.1.26 (2015-11-25)
-------------------
* Merge pull request `#112 <https://github.com/EndPointCorp/lg_ros_nodes/issues/112>`_ from EndPointCorp/ft-change_panos_via_button
  lg_sv: move forward if a button has been clicked
* Revert "WIP on changing panos pointing to the closest link"
  This reverts commit a38e0e036faeb6192c412b6bb075eaf5e53766c0.
  More work needs to be put into this commit before merging it.
* 1.1.25
* Generated new changelog
* Add titlecard to lg_sv panoviewer, fix fonts in lg_sv
* WIP on changing panos pointing to the closest link
* lg_sv: different buttons do different things
* pep8 fix
* lg_sv: move forward if a button has been clicked
* Contributors: Adam Vollrath, Jacob Minshall, Szymon Lipiński

1.1.24 (2015-11-16)
-------------------
* 1.1.23
* Generated changelogs
* Changed title for pano viewers
* lg_sv: remove 42-b hard coding in favor of parameterized ros url
* lg_sv: resets zoom after changing panos
* lg_sv: some changes for specific lgs
* lg_sv: optional zoom for streetview
  This really only works well if you're only using one screen. Kind of
  counter productive to the heart of a liquid galaxy, but hey, at least
  it's kind of in there now.
* lg_sv: parameterized zoom level
* lg_sv: parameterization for rosbridge url
* lg_sv: set initial pano via url
  Using panoid=foobar will set the initial pano to point to foobar.
* lg_sv: reset tilt/heading if none are specified
  Zoom also always reverts to the default max zoom out
* Contributors: Jacob Minshall, Wojciech Ziniewicz

1.1.23 (2015-11-13)
-------------------
* Changed title for pano viewers
* Contributors: Wojciech Ziniewicz

1.1.22 (2015-11-05)
-------------------
* Add titlecard to the lg_sv sv viewer
* panoviewer: allow images from the headnode
* Contributors: Jacob Minshall, Szymon Lipiński

1.1.21 (2015-10-22)
-------------------
* lg_sv: invert the heading given by the director
* 1.1.20
* Changelogs for 1.1.20
* init nearbypano parent class
* Contributors: Jacob Minshall, Matt Vollrath

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
* ADded dependency checking and fixed slots deserialization
* panoviewer: replay videos that are republished
* lg_sv: parameterize tilt
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

1.1.8 (2015-09-25)
------------------
* lg_sv: parameterize x_threshold
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath, Wojciech Ziniewicz

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
