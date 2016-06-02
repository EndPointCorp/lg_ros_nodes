^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spacenav_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.9 (2016-05-20)
------------------

Forthcoming
-----------

1.2.10 (2016-05-20)
-------------------
* 1.2.9
* Changelogs
* Contributors: Wojciech Ziniewicz

1.2.8 (2016-05-19)
------------------
* remove write_log_to_file imports
* fix tests by setting full_scale to 1
  The mid value without full scale is 0.5, so all of our messages were
  being ignored because they're > the mid value. Using full scale sets the
  mid value at 350/2
* add in missing abs calls to smooth out spacenav wrapper
* Contributors: Jacob Minshall

1.2.7 (2016-05-17)
------------------

1.2.6 (2016-05-16)
------------------
* remove unneeded run dependency
* Contributors: Jacob Minshall

1.2.5 (2016-05-12)
------------------
* add libudev-dev to run dependencies as well
* Contributors: Galaxy Admin, Jacob Minshall

1.2.4 (2016-05-10)
------------------
* fix spacenav_rezeroer by installing library
* Contributors: Jacob Minshall

1.2.3 (2016-05-06)
------------------
* Generated changelogs
* 1.2.2
* Contributors: Wojciech Ziniewicz

1.2.1 (2016-05-03)
------------------

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
