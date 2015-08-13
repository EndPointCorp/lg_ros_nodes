# -*- coding: utf-8 -
'''Event driven concurrent framework for Python'''
VERSION = (0, 9, 2, 'final', 0)

import os

from .utils.version import get_version

__version__ = version = get_version(VERSION)
__license__ = "BSD"
__author__ = "Luca Sbardella"
__contact__ = "luca.sbardella@gmail.com"
__homepage__ = "https://github.com/quantmind/pulsar"
__docformat__ = "restructuredtext"
CLASSIFIERS = ['Development Status :: 4 - Beta',
               'Environment :: Web Environment',
               'Intended Audience :: Developers',
               'License :: OSI Approved :: BSD License',
               'Operating System :: OS Independent',
               'Programming Language :: Python',
               'Programming Language :: Python :: 2',
               'Programming Language :: Python :: 2.7',
               'Programming Language :: Python :: 3',
               'Programming Language :: Python :: 3.3',
               'Programming Language :: Python :: 3.4',
               'Programming Language :: Python :: Implementation :: PyPy',
               'Topic :: Internet',
               'Topic :: Utilities',
               'Topic :: System :: Distributed Computing',
               'Topic :: Software Development :: Libraries :: Python Modules',
               'Topic :: Internet :: WWW/HTTP',
               'Topic :: Internet :: WWW/HTTP :: WSGI',
               'Topic :: Internet :: WWW/HTTP :: WSGI :: Server',
               'Topic :: Internet :: WWW/HTTP :: Dynamic Content']


DEFAULT_PORT = 8060
ASYNC_TIMEOUT = None
SERVER_NAME = 'pulsar'
JAPANESE = b'\xe3\x83\x91\xe3\x83\xab\xe3\x82\xb5\xe3\x83\xbc'.decode('utf-8')
CHINESE = b'\xe8\x84\x89\xe5\x86\xb2\xe6\x98\x9f'.decode('utf-8')
SERVER_SOFTWARE = "{0}/{1}".format(SERVER_NAME, version)

if os.environ.get('pulsar_setup_running') != 'yes':
    if os.environ.get('pulsar_speedup') == 'no':
        HAS_C_EXTENSIONS = False
    else:
        HAS_C_EXTENSIONS = True
        try:
            from .utils import lib
        except ImportError:
            HAS_C_EXTENSIONS = False

    from .utils.exceptions import *
    from .utils import system
    platform = system.platform
    from .utils.config import *
    from .async import *
    from .apps import *

    del get_version
    # Import data stores
    from .apps.data import stores
    del stores
    from .apps.data import data_stores
