#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lg_mirror'],
    package_dir={'': 'src'}
)

setup(**d)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
