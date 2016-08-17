#!/usr/bin/env python

import rospy
import unittest

from aggressive_proxy import ProxyHandler


PKG = 'aggressive_proxy'
NAME = 'test_proxy_handler'


class TestProxyHandler(unittest.TestCase):
    def setUp(self):
        self.a = 1

    def tearDown(self):
        self.a = 0

    def test_01_a_is_one(self):
        self.assertEqual(self.a, 1, "self.a should equal 1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProxyHandler)


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
