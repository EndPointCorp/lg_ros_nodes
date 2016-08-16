# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(
    packages=['aggressive_proxy'],
    package_dir={'': 'src'},
)

setup(**d)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
