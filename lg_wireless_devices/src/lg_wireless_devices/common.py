#!/usr/bin/env python3

import os
import configparser
import evdev
import argparse
import urllib.request, urllib.error, urllib.parse


def get_config(config_location):
    config = configparser.RawConfigParser()
    config.read(config_location)
    return config


def get_and_place_file(loc, dest):
    """
    Writes a remote or local file located at loc to dest
    """
    if loc[0] == '/':
        loc = "file://" + loc
    with open(dest, 'w') as f:
        f.write(urllib.request.urlopen(loc).read())
