#!/usr/bin/env python

import os
import shutil
import subprocess
import sys
import tempfile

import rospkg
from catkin_pkg.package import parse_package
import lg_builder


class Builder(object):
    def __init__(self):
        self.tempdir = None

    def _get_builddir(self):
        builddir = os.path.join(self.tempdir, 'build')
        if not os.path.exists(builddir):
            os.mkdir(builddir, mode=0755)

    def _write_debfiles(self, package):
        package_name = lg_builder.catkin_to_apt_name(package.name)

        builddir = self._get_builddir()

        debdir = os.path.join(builddir, 'debian')
        os.mkdir(debdir, mode=0755)

        control = lg_builder.generate_control(package)
        with open(os.path.join(debdir, 'control'), 'w') as f:
            f.write(control)

        changelog = lg_builder.generate_changelog(package)
        with open(os.path.join(debdir, 'changelog'), 'w') as f:
            f.write(control)

        compat = '9'
        with open(os.path.join(debdir, 'compat'), 'w') as f:
            f.write(compat)

        sourcedir = os.path.join(debdir, 'source')
        os.mkdir(sourcedir, mode=0755)

        source_format = '3.0 (quilt)'
        with open(os.path.join(sourcedir, 'format'), 'w') as f:
            f.write(source_format)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('lg_common')
        template_dir = os.path.join(pkg_path, 'templates')

        with open(os.path.join(template_dir, 'rules'), 'r') as f:
            rules_template = f.read()

        rules = rules_template.format(package_name=package_name)
        with open(os.path.join(debdir, 'rules'), 'w') as f:
            f.write(rules)

    def build(self, package_path):
        package_dir = lg_builder.get_package_path(package)

        package = parse_package(package_path)
        package.validate()

        self.tempdir = tempfile.mkdtemp()

        builddir = self._get_builddir()

        shutil.copytree(
            src=package_dir,
            dst=builddir,
            symlinks=True
        )

        self._write_debfiles(package)

        subprocess.check_output(
            args=[
                'dpkg-buildpackage',
                '-uc',
                '-us'
            ],
            cwd=builddir
        )

    def cleanup(self):
        if self.tempdir is None:
            return

        shutil.rmtree(self.tempdir)
        self.tempdir = None


if __name__ == '__main__':
    package_path = sys.argv[1]
    builder = Builder()
    try:
        builder.build(package_path)
    finally:
        builder.cleanup()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
