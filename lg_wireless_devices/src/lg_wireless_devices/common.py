#!/usr/bin/env python

import os
import ConfigParser
import evdev
import argparse
import urllib2


def get_config(config_location):
    config = ConfigParser.RawConfigParser()
    config.read(config_location)
    return config


def get_and_place_file(loc, dest):
    """
    Writes a remote or local file located at loc to dest
    """
    if loc[0] == '/':
        loc = "file://" + loc
    with open(dest, 'w') as f:
        f.write(urllib2.urlopen(loc).read())
