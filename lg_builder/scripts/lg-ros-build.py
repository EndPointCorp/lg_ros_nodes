#!/usr/bin/env python

import argparse
import glob
import os
import shutil
import subprocess
import tarfile
import tempfile

import rospkg
from catkin_pkg.package import parse_package
from lg_builder import RosBuilder


class Builder(object):
    def __init__(self, ros_distro, ros_os, ros_os_version):
        self._builder = RosBuilder(
            ros_distro=ros_distro,
            ros_os=ros_os,
            ros_os_version=ros_os_version,
        )
        self.tempdir = None

    def build(self, package_path):
        try:
            self._build(package_path)
        finally:
            self._cleanup()

    def _get_builddir(self):
        return os.path.join(self.tempdir, 'build')

    def _write_debfiles(self, package):
        package_name = self._builder.catkin_to_apt_name(package.name)

        builddir = self._get_builddir()

        debdir = os.path.join(builddir, 'debian')
        # XXX: remove legacy debian tree
        if os.path.exists(debdir):
            shutil.rmtree(debdir)

        os.mkdir(debdir, 0755)

        control = self._builder.generate_control(package)
        with open(os.path.join(debdir, 'control'), 'w') as f:
            f.write(control)

        changelog = self._builder.generate_changelog(package)
        with open(os.path.join(debdir, 'changelog'), 'w') as f:
            changelog.write_to_open_file(f)

        compat = '9'
        with open(os.path.join(debdir, 'compat'), 'w') as f:
            f.write(compat)

        sourcedir = os.path.join(debdir, 'source')
        os.mkdir(sourcedir, 0755)

        source_format = '3.0 (quilt)'
        with open(os.path.join(sourcedir, 'format'), 'w') as f:
            f.write(source_format)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('lg_builder')
        template_dir = os.path.join(pkg_path, 'templates')

        with open(os.path.join(template_dir, 'rules'), 'r') as f:
            rules_template = f.read()

        rules = rules_template.format(
            package_name=package_name,
            ros_distro=self._builder.ros_distro,
        )
        with open(os.path.join(debdir, 'rules'), 'w') as f:
            f.write(rules)

    def _build(self, package_path):
        package = parse_package(package_path)
        package_dir = self._builder.get_package_path(package)

        print "Building {} version {}".format(package.name, package.version)

        print "Validating package"
        package.validate()

        print "Updating rosdep"
        self._builder.rosdep_update()

        print "Checking rosdep sanity"
        self._builder.rosdep_sanity_check()

        # XXX: rosdep only resolves public dependencies
        # print "Installing dependencies"
        # self._builder.rosdep_install_deps(package_dir)

        print "Creating tempdir"
        self.tempdir = tempfile.mkdtemp()

        print "Copying sources to tempdir"
        builddir = self._get_builddir()
        shutil.copytree(
            src=package_dir,
            dst=builddir,
            symlinks=True
        )

        print "Writing debian metadata"
        self._write_debfiles(package)

        print "Creating upstream tarball"
        debian_name = self._builder.catkin_to_apt_name(package.name)
        upstream_filename = os.path.join(
            self.tempdir,
            '{}_{}.orig.tar.gz'.format(debian_name, package.version)
        )
        with tarfile.open(upstream_filename, 'w:gz') as tar:
            tar.add(builddir, arcname='')

        print "Running dpkg-buildpackage"
        subprocess.check_output(
            args=[
                'dpkg-buildpackage',
                '-d',
                '-uc',
                '-us'
            ],
            cwd=builddir
        )

        print "Copying deb package to {}".format(os.getcwd())
        for f in glob.glob(os.path.join(self.tempdir, '*.deb')):
            shutil.copy(f, os.getcwd())

    def _cleanup(self):
        if self.tempdir is None:
            return

        shutil.rmtree(self.tempdir)
        self.tempdir = None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Build a ROS package.')
    parser.add_argument('package_path')
    parser.add_argument('--ros_distro', default='indigo')
    parser.add_argument('--os', default='ubuntu')
    parser.add_argument('--os_version', default='trusty')
    args = parser.parse_args()

    package_path = args.package_path
    ros_distro = args.ros_distro
    ros_os = args.os
    ros_os_version = args.os_version

    builder = Builder(
        ros_distro=ros_distro,
        ros_os=ros_os,
        ros_os_version=ros_os_version,
    )
    builder.build(package_path)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
