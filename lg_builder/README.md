lg\_builder
-----------

Debian packaging for ROS packages.

This package installs the `lg-ros-build` script which is used to build debian packages.

Presently it is set to build for ROS indigo and Ubuntu trusty.

Untracked system dependencies for build:
* `debhelper`
* `fakeroot`

Usage: `lg-ros-build <path_to_package>`

The resulting .deb package will be copied to your cwd.

