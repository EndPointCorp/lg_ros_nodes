#!/usr/bin/env python

# Functional tests for an aggressive HTTP proxy.

# Authored by Adam Vollrath <adam@endpoint.com>

import rospy
import urllib
import unittest

from lg_common.helpers import write_log_to_file

NAME = 'test_aggressive_proxy'
PKG = 'aggressive_proxy'

class TestAggressiveProxy(unittest.TestCase):
    def setUp(self):
        rospy.sleep(2)

    def test_01_hello_world(self):
        upstream_socket = rospy.get_param('~upstream_socket')
        r = urllib.urlopen('http://%s' % upstream_socket)
        self.assertEqual(r.code, 404)

if __name__ == '__main__':
    import rostest

    rospy.init_node('aggressive_tester')

    rostest.rosrun(PKG, NAME, TestAggressiveProxy)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
