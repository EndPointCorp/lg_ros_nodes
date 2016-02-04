^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_cms_director
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.20 (2015-10-21)
-------------------

Forthcoming
-----------

1.1.32 (2016-01-28)
-------------------

1.1.31 (2016-01-20)
-------------------

1.1.30 (2016-01-11)
-------------------

1.1.29 (2016-01-04)
-------------------
* pep8: used proper ignoring of lg_cms_director/src directory
  Pep8 can now be run from the root directory w/o needing to run again in
  the lg_cms_director directory. The pep8 config in lg_cms_director has
  been removed and the test runner has been changed plus generalized (the
  removal of the project name lg_ros_nodes).
  Lastly we are now ignoring the pep8 E402. This fails in later versions
  of pep8, and only deals with the position of imports in the file. When
  we want to deal with this, we can remove it from the ignores.
* Contributors: Jacob Minshall

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
* pulsar: fixes to pulsar
* Contributors: Jacob Minshall

1.1.14 (2015-10-08)
-------------------

1.1.13 (2015-10-08)
-------------------

1.1.12 (2015-10-07)
-------------------
* pep8: ignore E265, block comments requiring space after #
* Contributors: Jacob Minshall

1.1.11 (2015-10-06)
-------------------

1.1.10 (2015-10-05)
-------------------
* Added lots of docs

1.1.9 (2015-09-25)
------------------
* PEP8'ed director
* Updated changelogs
* Revert director.py to where it was at 8222d7b6b38f170b7c6a7f379909182fe297b56d
* lg_cms_director: fix pulsar/trollius setup
  find_packages was doing something weird and didn't want to use the cruft-generating setuptools version.
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath

1.1.8 (2015-09-25)
------------------
* Contributors: Adam Vollrath, Matt Vollrath

1.1.7 (2015-09-24)
------------------
* PEP8'ed director
* Contributors: Adam Vollrath

1.1.6 (2015-09-24)
------------------
* Revert director.py to where it was at 8222d7b6b38f170b7c6a7f379909182fe297b56d
* Contributors: Adam Vollrath

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
* Revert to previous packaging of director's dependencies thx to @mvollrath
* Contributors: Wojciech Ziniewicz

1.1.0 (2015-09-17)
------------------
* Contributors: Jacob Minshall, Matt Vollrath

1.0.9 (2015-09-09)
------------------

1.0.8 (2015-08-12)
------------------
* Properly package director dependencies
* Contributors: Matt Vollrath

1.0.7 (2015-08-12)
------------------
* Add director and IS msg packages
* Contributors: Matt Vollrath

1.0.6 (2015-08-10)
------------------

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

0.0.1 (2015-07-08)
------------------
