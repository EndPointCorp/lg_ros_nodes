#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'trollius',
        'pulsar',
        'pulsar.apps',
        'pulsar.async',
        'pulsar.utils',
        'pulsar.apps.tasks',
        'pulsar.apps.ds',
        'pulsar.apps.http',
        'pulsar.apps.greenio',
        'pulsar.apps.ws',
        'pulsar.apps.pulse',
        'pulsar.apps.shell',
        'pulsar.apps.data',
        'pulsar.apps.tx',
        'pulsar.apps.socket',
        'pulsar.apps.wsgi',
        'pulsar.apps.test',
        'pulsar.apps.rpc',
        'pulsar.utils.structures',
        'pulsar.utils.tools',
        'pulsar.utils.system',
        'pulsar.utils.settings',
        'pulsar.apps.pulse.management',
        'pulsar.apps.data.odm',
        'pulsar.apps.data.stores',
        'pulsar.apps.test.plugins',
        'pulsar.apps.pulse.management.commands',
        'pulsar.apps.data.stores.sql',
        'pulsar.apps.data.stores.couchdb',
        'pulsar.apps.data.stores.redis',
        'pulsar.apps.data.stores.pulsards'
    ],
    package_dir={'': 'src'}
)

setup(**d)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
