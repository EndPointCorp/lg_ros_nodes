'''Operative system specific functions and classes.
'''
import os

from .runtime import Platform

platform = Platform()
seconds = platform.seconds

from .base import *

if platform.type == 'win':    # pragma nocover
    from .windowssystem import *
elif platform.is_appengine:    # pragma nocover
    from .appengine import *
else:
    from .posixsystem import *


try:
    import psutil
except ImportError:    # pragma    nocover
    psutil = None

try:
    import ujson as json
except ImportError:     # pragma    nocover
    import json


memory_symbols = ('K', 'M', 'G', 'T', 'P', 'E', 'Z', 'Y')
memory_size = dict(((s, 1 << (i+1)*10) for i, s in enumerate(memory_symbols)))


def convert_bytes(b):
    '''Convert a number of bytes into a human readable memory usage, bytes,
kilo, mega, giga, tera, peta, exa, zetta, yotta'''
    if b is None:
        return '#NA'
    for s in reversed(memory_symbols):
        if b >= memory_size[s]:
            value = float(b) / memory_size[s]
            return '%.1f%sB' % (value, s)
    return "%sB" % b


def process_info(pid=None):
    '''Returns a dictionary of system information for the process ``pid``.

    It uses the psutil_ module for the purpose. If psutil_ is not available
    it returns an empty dictionary.

    .. _psutil: http://code.google.com/p/psutil/
    '''
    if psutil is None:  # pragma    nocover
        return {}
    pid = pid or os.getpid()
    try:
        p = psutil.Process(pid)
    # this fails on platforms which don't allow multiprocessing
    except psutil.NoSuchProcess:  # pragma    nocover
        return {}
    else:
        mem = p.memory_info()
        return {'memory': mem.rss,
                'memory_virtual': mem.vms,
                'cpu_percent': p.cpu_percent(),
                'nice': p.nice(),
                'num_threads': p.num_threads()}
