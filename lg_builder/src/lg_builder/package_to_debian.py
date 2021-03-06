import datetime
import os.path
import subprocess
import time

from catkin_pkg.package import parse_package
from catkin_pkg import changelog as catkin_changelog
from debian.changelog import Changelog, Version

DEBIAN_STANDARDS_VERSION = '3.9.2'


class RosBuilder:
    def __init__(self, ros_distro, ros_os, ros_os_version):
        self.ros_distro = ros_distro
        self.ros_os = ros_os
        self.ros_os_version = ros_os_version

    def rosdep_update(self):
        """Update the rosdep database."""
        rosdep_cmd = [
            'rosdep',
            'update'
        ]
        subprocess.check_output(rosdep_cmd)

    def rosdep_sanity_check(self):
        """Make sure rosdep is working."""
        rosdep_cmd = [
            'rosdep',
            'resolve',
            'rospy',  # this name should always resolve
            '--os={}:{}'.format(self.ros_os, self.ros_os_version),
            '--rosdistro={}'.format(self.ros_distro)
        ]
        try:
            subprocess.check_output(
                rosdep_cmd
            )
        except Exception:
            return False
        return True

    def rosdep_install_deps(self, package_path):
        """Install all dependencies for the package at the given path."""
        rosdep_cmd = [
            'rosdep',
            'install',
            '--from-paths',
            package_path,
            '--os={}:{}'.format(self.ros_os, self.ros_os_version),
            '--rosdistro={}'.format(self.ros_distro),
            '-y'
        ]
        subprocess.check_output(rosdep_cmd)

    def resolve_rosdep(self, catkin_name):
        """Resolve a catkin package name to an APT package name.

        If rosdep can't find the package, a name will be made up.

        Args:
            catkin_name (str)

        Returns:
            set(str): APT package names.
        """
        rosdep_cmd = [
            'rosdep',
            'resolve',
            str(catkin_name),
            '--os={}:{}'.format(self.ros_os, self.ros_os_version),
            '--rosdistro={}'.format(self.ros_distro)
        ]
        try:
            with open(os.devnull, 'wb') as devnull:
                rosdep_output = subprocess.check_output(
                    rosdep_cmd,
                    stderr=devnull
                )

            assert rosdep_output.splitlines()[0] == '#apt'
            deb_names = rosdep_output.splitlines()[1].split()
        except Exception:
            deb_names = [self.catkin_to_apt_name(catkin_name)]
        return set(deb_names)

    def catkin_to_apt_name(self, catkin_name):
        """Format a package name as if it were part of the distro.

        Args:
            catkin_name (str)

        Returns:
            str: Made up package name.
        """
        return 'ros-{distro}-{name}'.format(
            distro=self.ros_distro,
            name=str(catkin_name).replace('_', '-')
        )

    def get_build_depends(self, package):
        """Get list of deb package build depends.

        Args:
            package (catkin_pkg.package.Package)

        Returns:
            list(str): Build dependencies.
        """
        build_depends = list(map(
            self.resolve_rosdep,
            list(set(package.buildtool_depends) | set(package.build_depends))
        ))
        build_depends = [d for s in build_depends for d in s]

        build_depends.insert(0, 'debhelper (>= 9.0.0)')

        return build_depends

    def get_run_depends(self, package):
        """Get list of deb package run depends.

        Args:
            package (catkin_pkg.package.Package)

        Returns:
            list(str): Run dependencies.
        """
        run_depends = list(map(
            self.resolve_rosdep,
            package.run_depends
        ))
        run_depends = [d for s in run_depends for d in s]

        run_depends.insert(0, '${shlibs:Depends}, ${misc:Depends}')

        return run_depends

    def generate_control(self, package):
        """Generate Debian control file for a catkin package.

        Args:
            package (catkin_pkg.package.Package)

        Returns:
            str: Contents of the Debian control file.
        """
        control = []

        source = self.catkin_to_apt_name(package.name)

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

        build_depends = self.get_build_depends(package)

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

        run_depends = self.get_run_depends(package)

        control.append('Depends: {}'.format(
            ', '.join(run_depends)
        ))

        description = package.description

        control.append('Description: {}'.format(description))

        return '\n'.join(control)

    def generate_changelog(self, package):
        """Generate a Debian changelog for the catkin package.

        Args:
            package (catkin_pkg.package.Package)

        Returns:
            debian.Changelog
        """
        package_path = self.get_package_path(package)
        package_changelog = catkin_changelog.get_changelog_from_path(package_path)

        debian_changelog = Changelog()

        package_name = self.catkin_to_apt_name(package.name)
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
            full_version = version + '-0' + self.ros_os_version
            debian_changelog.new_block(
                package=package_name,
                version=Version(full_version),
                distributions=self.ros_os_version,
                urgency='high',
                author=maintainer,
                date=self._datetime_to_rfc2822(date),
            )
            debian_changelog.add_change('')
            list(map(transfer_version_change, changes))
            debian_changelog.add_change('')

        if package_changelog is not None:
            list(map(transfer_version_block, package_changelog.foreach_version()))
        else:
            # If CHANGELOG.rst is missing, generate a bare changelog
            version = str(package.version)
            date = datetime.datetime.now()
            changes = []
            block = (version, date, changes)
            transfer_version_block(block)

        return debian_changelog

    def get_package_path(self, package, sub_path=None):
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

    def _datetime_to_rfc2822(self, dt):
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

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
