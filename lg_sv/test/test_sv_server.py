#!/usr/bin/env python

PKG = 'lg_sv'
NAME = 'test_sv_server'
LAT = '43.726319'
LON = '12.636377'

import sys
import rospy
import rostest
import unittest
from lg_sv import StreetviewUtils

class TestSVServer(unittest.TestCase):
    def setUp(self):
        pass
    
    def tearDown(self):
        pass

    def test_sv_utils(self):
        sv_utils = StreetviewUtils()
        response = sv_utils.get_panoid_from_lat_lon(LAT, LON)
        self.assertIsInstance(response, str)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSVServer, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
