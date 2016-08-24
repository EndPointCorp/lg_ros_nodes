#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lg_common'],
    package_dir={'': 'src'},
    package_data={'lg_common': ['extensions/ros_window_ready/*']},
    scripts=['bin/lg-code-to-command'],
    requires=[]
)

setup(**d)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
