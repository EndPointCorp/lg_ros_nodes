#!/usr/bin/env python


import rospy
import urllib
import unittest

NAME = 'test_aggressive_proxy'
PKG = 'aggressive_proxy'

class TestAggressiveProxy(unittest.TestCase):
    def setUp(self):
        rospy.sleep(2)

    def test_01_hello_world(self):
        r = urllib.urlopen('http://localhost:9900')
        self.assertEqual(r.code, 404)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAggressiveProxy)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
