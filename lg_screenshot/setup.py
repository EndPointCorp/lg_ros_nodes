#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lg_screenshot'],
    package_dir={'': 'src'},
    package_data={
        'lg_screenshot': [
            'webapps/*'
        ]
    },
    scripts=['bin/lg-screenshot'],
    requires=[]
)

setup(**d)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
