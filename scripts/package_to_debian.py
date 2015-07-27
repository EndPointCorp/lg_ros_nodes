#!/usr/bin/env python

import datetime
import os.path
import subprocess
import sys
import time

from catkin_pkg.package import parse_package
from catkin_pkg import changelog as catkin_changelog
from debian.changelog import Changelog, Version

ROS_DISTRO = 'indigo'
ROS_OS = 'ubuntu'
ROS_OS_VERSION = 'trusty'
DEBIAN_STANDARDS_VERSION = '3.9.2'


def rosdep_update():
    """Update the rosdep database."""
    rosdep_cmd = [
        'rosdep',
        'update'
    ]
    subprocess.check_output(rosdep_cmd)


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


def debianize(package_path):
    """Create debian artifacts for a catkin package at the given path.

    Args:
        package_path (str): Path to the package root or its package.xml.
    """
    package = parse_package(package_path)
    package.validate()

    rosdep_update()
    if not rosdep_sanity_check():
        raise Exception('rosdep sanity check failed! is rosdep installed?')

    control = generate_control(package)
    print control
    print

    changelog = generate_changelog(package)
    print changelog
    print


def generate_control(package):
    """Generate Debian control file for a catkin package.

    Args:
        package (catkin_pkg.package.Package)

    Returns:
        str: Contents of the Debian control file.
    """
    control = []

    source = _catkin_to_apt_name(package.name)

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
        _catkin_to_apt_name,
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
        _catkin_to_apt_name,
        package.run_depends
    )

    run_depends.insert(0, '${shlibs:Depends}, ${misc:Depends}')

    control.append('Depends: {}'.format(
        ', '.join(run_depends)
    ))

    description = package.description

    control.append('Description: {}'.format(description))

    #replaces = '__REPLACES__'

    #control.append('Replaces: {}'.format(replaces))

    #conflicts = '__REPLACES__'

    #control.append('Conflicts: {}'.format(conflicts))

    return '\n'.join(control)


def generate_changelog(package):
    """Generate a Debian changelog for the catkin package.

    Args:
        package (catkin_pkg.package.Package)

    Returns:
        debian.Changelog
    """
    package_path = _get_package_path(package)
    package_changelog = catkin_changelog.get_changelog_from_path(package_path)

    debian_changelog = Changelog()

    package_name = _catkin_to_apt_name(package.name)
    maintainer = '{} <{}>'.format(
        package.maintainers[0].name,
        package.maintainers[0].email
    )

    def transfer_version_change(change):
        debian_changelog.add_change(str(change))

    def transfer_version_block(v):
        version = v[0]
        date = v[1]
        changes = v[2]
        full_version = version + '-0' + ROS_OS_VERSION
        debian_changelog.new_block(
            package=package_name,
            version=Version(full_version),
            distributions=ROS_OS_VERSION,
            urgency='high',
            author=maintainer,
            date=_datetime_to_rfc2822(date),
        )
        debian_changelog.add_change('')
        map(transfer_version_change, changes)
        debian_changelog.add_change('')

    if package_changelog is not None:
        map(transfer_version_block, package_changelog.foreach_version())
    else:
        # If CHANGELOG.rst is missing, generate a phony one
        version = str(package.version)
        date = datetime.datetime.now()
        changes = []
        block = (version, date, changes)
        transfer_version_block(block)

    return debian_changelog


def _datetime_to_rfc2822(dt):
    """Convert a datetime to an RFC 2822 date.

    This is the date format specified for Debian changelogs.

    Args:
        dt (datetime.datetime)

    Returns:
        str: RFC 2822 date/time string.
    """
    from email import utils
    timestamp = time.mktime(dt.timetuple())
    return utils.formatdate(timestamp)


def _get_debian_path(package, sub_path=None):
    """Get the path of the package's debian meta, or a path relative to it.

    Args:
        package (catkin_pkg.package.Package)
        sub_path (str): Optional path relative to the debian path.

    Returns:
        str: Debian path for package.
    """
    path = os.path.abspath(_get_package_path(package, 'debian'))
    if sub_path is not None:
        path = os.path.join(path, sub_path)
    return path


def _get_package_path(package, sub_path=None):
    """Get the package path, or a path relative to it.

    Args:
        package (catkin_pkg.package.Package)
        sub_path (str): Optional path relative to the package path.

    Returns:
        str: Package path.
    """
    path = os.path.abspath(os.path.join(package.filename, os.pardir))
    if sub_path is not None:
        path = os.path.join(path, sub_path)
    return path


def _catkin_to_apt_name(catkin_name):
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


if __name__ == '__main__':
    debianize(sys.argv[1])

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
