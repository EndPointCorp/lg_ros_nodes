^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_replay
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.20 (2015-10-21)
-------------------

1.1.29 (2016-01-04)
-------------------

1.1.28 (2015-12-10)
-------------------

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

1.1.11 (2015-10-06)
-------------------

1.1.10 (2015-10-05)
-------------------
* lg_replay: update to test for non categorized event publishing
* lg_replay: optimize message publishing
  - removed `if self.event_code` since there's a default event code
  supplied by the __init_\_ method
  - changed the constant getattr(ecodes, event_code) to a instance
  variable
  - removed the categorize(event) since a pretty message isn't needed
  There is now no longer a large queue build up, at least on my machine...
* lg_replay: add missing import
* Added lots of docs

1.1.9 (2015-09-25)
------------------
* lg_replay: retain permissions on other event devices
* lg_replay: exception handling
* lg_replay: quietly handle unplugged device
* lg_replay: add import and name input device device
* lg_replay: allow path to be supplied in place of name
* Contributors: Adam Vollrath, Jacob Minshall

1.1.8 (2015-09-25)
------------------
* Contributors: Adam Vollrath

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
* Initial package with test coverage for lg_replay
* Contributors: Wojciech Ziniewicz

1.0.5 (2015-08-03)
------------------

1.0.4 (2015-07-31)
------------------

1.0.3 (2015-07-29 19:30)
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
