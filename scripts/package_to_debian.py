#!/usr/bin/env python

import subprocess
import sys

from catkin_pkg.package import parse_package

ROS_DISTRO = 'indigo'
ROS_OS = 'ubuntu'
ROS_OS_VERSION = 'trusty'
DEBIAN_STANDARDS_VERSION = '3.9.2'

def rosdep_sanity_check():
    """Make sure rosdep is working."""
    rosdep_cmd = [
        'rosdep',
        'resolve',
        'rospy',  # this name should always resolve
        '--os={}:{}'.format(ROS_OS, ROS_OS_VERSION),
        '--rosdistro={}'.format(ROS_DISTRO)
    ]
    try:
        subprocess.check_output(
            rosdep_cmd
        )
    except:
        return False
    return True


def catkin_to_apt_name(catkin_name):
    """Resolve a catkin package name to an APT package name.

    If the package is part of the ROS distro, rosdep will resolve the name.

    Otherwise, the name will be generated.

    Args:
        catkin_name (str)

    Returns:
        str: APT package name.
    """
    rosdep_cmd = [
        'rosdep',
        'resolve',
        str(catkin_name),
        '--os={}:{}'.format(ROS_OS, ROS_OS_VERSION),
        '--rosdistro={}'.format(ROS_DISTRO)
    ]
    try:
        with open(os.devnull, 'wb') as devnull:
            rosdep_output = subprocess.check_output(
                rosdep_cmd,
                stderr=devnull
            )

        assert rosdep_output.splitlines()[0] == '#apt'
        deb_name = rosdep_output.splitlines()[1]
    except:
        # Fall back to a dumb string format.
        deb_name = 'ros-{distro}-{name}'.format(
            distro=ROS_DISTRO,
            name=str(catkin_name).replace('_', '-')
        )
    return deb_name

def debianize(package_path):
    """Create debian artifacts for a catkin package at the given path.

    Args:
        package_path (str): Path to the package root or its package.xml.
    """
    package = parse_package(package_path)
    package.validate()

    if not rosdep_sanity_check():
        raise Exception('rosdep sanity check failed! is rosdep installed?')

    control = generate_control(package)
    print control

def generate_control(package):
    """Generate Debian control file for a catkin package.

    Args:
        package (catkin_pkg.package.Package)

    Returns:
        str: Contents of the Debian control file.
    """
    control = []

    source = catkin_to_apt_name(package.name)

    control.append('Source: {}'.format(source))

    section = 'misc'

    control.append('Section: {}'.format(section))

    priority = 'extra'

    control.append('Priority: {}'.format(priority))

    maintainer = '{name} <{email}>'.format(
        name=package.maintainers[0].name,
        email=package.maintainers[0].email
    )

    control.append('Maintainer: {}'.format(maintainer))

    build_depends = map(
        catkin_to_apt_name,
        list(set(package.buildtool_depends) | set(package.build_depends))
    )

    # debhelper Build-Depends is arbitrary.
    build_depends.insert(0, 'debhelper (>= 9.0.0)')

    control.append('Build-Depends: {}'.format(
        ', '.join(build_depends)
    ))

    assert len(package.urls) > 0, "A url is required in package.xml."
    homepage = str(package.urls[0])

    control.append('Homepage: {}'.format(homepage))

    standards_version = DEBIAN_STANDARDS_VERSION

    control.append('Standards-Version: {}'.format(standards_version))
    control.append('')
    control.append('Package: {}'.format(source))
    control.append('Architecture: any')

    run_depends = map(
        catkin_to_apt_name,
        package.run_depends
    )

    run_depends.insert(0, '${shlibs:Depends}, ${misc:Depends}')

    control.append('Depends: {}'.format(
        ', '.join(run_depends)
    ))

    description = package.description

    control.append('Description: {}'.format(description))

    replaces = '__REPLACES__'

    control.append('Replaces: {}'.format(replaces))

    conflicts = '__REPLACES__'

    control.append('Conflicts: {}'.format(conflicts))

    return '\n'.join(control)

if __name__ == '__main__':
    debianize(sys.argv[1])

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
