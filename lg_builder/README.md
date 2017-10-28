lg\_builder
-----------

Debian packaging for ROS packages.

This package installs the `lg-ros-build` script which is used to build debian packages.

Untracked system dependencies for build:
* `debhelper`
* `fakeroot`

Usage: `lg-ros-build <path_to_package> --ros_distro=<distro> --os=<os> --os_version=<os_distro>`

e.g. `lg-ros-build /path/to/lg_common --ros_distro=kinetic --os=ubuntu --os_version=xenial`

Check `lg-ros-build --help` for help.

The resulting .deb package will be copied to your cwd.

