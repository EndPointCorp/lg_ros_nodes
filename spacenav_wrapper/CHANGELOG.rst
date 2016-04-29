^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spacenav_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2016-04-29)
------------------

1.1.50 (2016-04-27)
-------------------

1.1.49 (2016-04-26)
-------------------

1.1.48 (2016-04-20)
-------------------

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

1.1.43 (2016-04-14)
-------------------

1.1.42 (2016-04-14)
-------------------
* readme for spacenav_wrapper
* spacenav_wrapper: log and pep8 fixes
* spacenav_wrapper: parameterize more things
  Gutter value, buffer size, and whether or not we should actually
  relaunch. This also has defaults now that leaves the spacenav_wrapper as
  nothing more than an echo to /spacenav/twist. This is good in case
  problems arise with the spacenav wrapper.
* improvements to spacenav_wrapper
* spacenav-rezero: turned into rosnode
  NOTE: ran into issues building this in Jade, or possibly just on my
  system, but inside the docker tester this builds properly.
* building spacenav-rezero in ros package now
* spacenav_wrapper: initial package
  Supports parameterized gutter vals, and has a relaunch check.
* Contributors: Jacob Minshall
