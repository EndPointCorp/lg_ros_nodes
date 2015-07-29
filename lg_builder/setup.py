#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lg_builder'],
    package_dir={'': 'src'},
    scripts=['scripts/lg-ros-build'],
    requires=['catkin_pkg', 'python-debian']
)

setup(**d)
