#!/usr/bin/env python

PKG = 'lg_sv'
NAME = 'test_sv_server'
LAT = '43.726319'
LON = '12.636377'

TILT_MAX = 80
TILT_MIN = -80
NAV_SENSITIVITY = 1
NAV_INTERVAL = .1

import sys
import rospy
import rostest
import unittest
from lg_sv import StreetviewUtils, StreetviewServer

class MockPublisher:
    def __init__(self):
        self.data = []

    def publish(self, data):
        self.data.append(data)

class TestSVServer(unittest.TestCase):
    def setUp(self):
        self.location_pub = MockPublisher()
        self.pano_pub = MockPublisher()
        self.pov_pub = MockPublisher()
        self.server = StreetviewServer(
             self.location_pub, self.pano_pub, self.pov_pub, TILT_MAX,
             TILT_MIN, NAV_SENSITIVITY, NAV_INTERVAL)
    
    def tearDown(self):
        pass

    def test_sv_utils(self):
        response = StreetviewUtils.get_panoid_from_lat_lon(LAT, LON)
        self.assertIsInstance(response, str)

    def test_1_pano_pub(self):
        # test initial state
        self.assertEqual(len(self.pano_pub.data), 0,
                         'pano_pub data should not have anything in it')
        self.assertEqual(self.server.panoid, str(),
                         'panoid should not be set yet')
        new_pano = 'foo_bar'
        self.server.pub_panoid(new_pano)
        # test final state
        self.assertEqual(len(self.pano_pub.data), 1,
                         'pano_pub data did not have the correct amount of data')
        self.assertEqual(self.server.panoid, new_pano,
                         'panoid should not be set yet')
        self.assertEqual(self.pano_pub.data[0], new_pano,
                         'pano_pub data did not have the correct value')


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSVServer, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
