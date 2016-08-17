#!/usr/bin/env python

# Functional tests for an aggressive HTTP proxy.

# Authored by Adam Vollrath <adam@endpoint.com>

import json

import rospy
import urllib
import unittest

from lg_common.helpers import write_log_to_file

NAME = 'test_aggressive_proxy'
PKG = 'aggressive_proxy'

class TestAggressiveProxy(unittest.TestCase):
    def setUp(self):
        rospy.sleep(1)
        self.upstream_socket = rospy.get_param('~upstream_socket')

    def test_01_hello_world(self):
        r = urllib.urlopen('http://%s/null.html' % self.upstream_socket)
        self.assertEqual(r.code, 404)

    def test_02_fast_response(self):
        # One tenth of a second
        a = urllib.urlopen('http://%s/delay/00100.html' % self.upstream_socket)
        a.read()
        b = urllib.urlopen('http://%s/delay/00100.html' % self.upstream_socket)
        b.read()
        c = urllib.urlopen('http://%s/delay/00100.html' % self.upstream_socket)
        c.read()
        r = urllib.urlopen('http://%s/delay/00100.html' % self.upstream_socket)

        # We should get back a JSON list of request timestamps.
        timestamps = json.loads(r.read())
        write_log_to_file(str(timestamps))
        self.assertEqual(r.code, 200)

        # We made four requests and none should have retried.
        self.assertEqual(len(timestamps), 4)

    def test_03_slower_response(self):
        # Two and a half seconds
        r = urllib.urlopen('http://%s/delay/02500.html' % self.upstream_socket)

        # We should get back a JSON list of request timestamps.
        timestamps = json.loads(r.read())
        write_log_to_file(str(timestamps))
        self.assertEqual(r.code, 200)

        # There should be three requests made within 2.5 seconds.
        self.assertEqual(len(timestamps), 3)

if __name__ == '__main__':
    import rostest

    rospy.init_node('aggressive_tester')

    rostest.rosrun(PKG, NAME, TestAggressiveProxy)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
