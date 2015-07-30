#!/usr/bin/env python

import glob
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile

import rospkg
from catkin_pkg.package import parse_package
import lg_builder


class Builder(object):
    def __init__(self):
        self.tempdir = None

    def build(self, package_path):
        try:
            self._build(package_path)
        finally:
            self._cleanup()

    def _get_builddir(self):
        return os.path.join(self.tempdir, 'build')

    def _write_debfiles(self, package):
        package_name = lg_builder.catkin_to_apt_name(package.name)

        builddir = self._get_builddir()

        debdir = os.path.join(builddir, 'debian')
        # XXX: remove legacy debian tree
        if os.path.exists(debdir):
            shutil.rmtree(debdir)

        os.mkdir(debdir, 0755)

        control = lg_builder.generate_control(package)
        with open(os.path.join(debdir, 'control'), 'w') as f:
            f.write(control)

        changelog = lg_builder.generate_changelog(package)
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

        rules = rules_template.format(package_name=package_name)
        with open(os.path.join(debdir, 'rules'), 'w') as f:
            f.write(rules)

    def _build(self, package_path):
        package = parse_package(package_path)
        package_dir = lg_builder.get_package_path(package)

        print "Building {} version {}".format(package.name, package.version)

        print "Validating package"
        package.validate()

        print "Updating rosdep"
        lg_builder.rosdep_update()

        print "Checking rosdep sanity"
        lg_builder.rosdep_sanity_check()

        # XXX: rosdep only resolves public dependencies
        # print "Installing dependencies"
        # lg_builder.rosdep_install_deps(package_dir)

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
        debian_name = lg_builder.catkin_to_apt_name(package.name)
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
    package_path = sys.argv[1]
    builder = Builder()
    builder.build(package_path)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
