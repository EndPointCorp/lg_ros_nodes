^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lg_cms_director
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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

1.1.26 (2015-11-25)
-------------------

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
* fixes to pulsar
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
* lg_cms_director: fix pulsar/trollius setup
  find_packages was doing something weird and didn't want to use the cruft-generating setuptools version.
* Contributors: Adam Vollrath, Jacob Minshall, Matt Vollrath

1.1.8 (2015-09-25)
------------------

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
