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
import json
import rospy
import rostest
import unittest
from lg_sv import StreetviewUtils, StreetviewServer
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import String

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

    def test_sv_utils_metadata(self):
        response = StreetviewUtils.get_metadata_from_lat_lon(LAT, LON)
        self.assertIsInstance(response, dict)
        self.assertTrue('Location' in response)
        self.assertTrue('panoId' in response['Location'])


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
                         'server.panoid data did not have the correct value')
        self.assertEqual(self.pano_pub.data[0], new_pano,
                         'pano_pub data did not have the correct value')

    def test_2_pov_pub(self):
        # test initial state
        self.assertEqual(len(self.pov_pub.data), 0,
                         'pov_pub data should not have anything in it')
        self.assertEqual(self.server.pov, Quaternion(),
                         'pov should not be set yet')
        new_pov = Quaternion(x=1.0, y=1.0, z=1.0, w=1.0)
        self.server.pub_pov(new_pov)
        # test final state
        self.assertEqual(len(self.pov_pub.data), 1,
                         'pov_pub data did not have the correct amount of data')
        self.assertEqual(self.server.pov, new_pov,
                         'server.pov data did not have the correct value')
        self.assertEqual(self.pov_pub.data[0], new_pov,
                         'pov_pub data did not have the correct value')

    def test_3_location_pub(self):
        # test initial state
        self.assertEqual(len(self.location_pub.data), 0,
                         'location_pub data should not have anything in it')
        self.assertEqual(self.server.location, Pose2D(),
                         'location should not be set yet')
        new_location = Pose2D(x=1.0, y=1.0, theta=1.0)
        self.server.pub_location(new_location)
        # test final state
        self.assertEqual(len(self.location_pub.data), 1,
                         'location_pub data did not have the correct amount of data')
        self.assertEqual(self.server.location, new_location,
                         'server.location data did not have the correct value')
        self.assertEqual(self.location_pub.data[0], new_location,
                         'location_pub data did not have the correct value')

    def test_4_handle_location_msg(self):
        # test initial state
        self.assertEqual(len(self.pano_pub.data), 0,
                         'pano_pub data should not have anything in it')
        self.assertEqual(self.server.panoid, str(),
                         'server.panoid should not be set yet')

        new_location = Pose2D(x=LAT, y=LON, theta=1.0)
        expected_panoid = StreetviewUtils.get_panoid_from_lat_lon(LAT, LON)
        self.server.handle_location_msg(new_location)

        # test final state
        self.assertEqual(len(self.pano_pub.data), 1,
                         'pano_pub data did not have the correct amount of data')
        self.assertEqual(self.server.panoid, expected_panoid,
                         'server.panoid data did not have the correct value')
        self.assertEqual(self.pano_pub.data[0], expected_panoid,
                         'pano_pub data did not have the correct value')

    def test_5_handle_metadata_msg(self):
        # test initial state
        self.assertEqual(self.server.get_metadata(), None,
                         'metadata should start as none')
        self.assertEqual(self.server.nearby_panos.panoid, None,
                         'nearby_panos.panoid should start as None')

        metadata = StreetviewUtils.get_metadata_from_lat_lon(LAT, LON)
        metadata = StreetviewUtils.translate_server_metadata_to_client_form(metadata)
        msg = String(json.dumps(metadata))
        # testing a metadata msg when nearby_panos panoid is none, should be ignored
        self.server.handle_metadata_msg(msg)
        self.assertEqual(self.server.get_metadata(), None,
                         'metadata should still be none')
        self.assertEqual(self.server.nearby_panos.panoid, None,
                         'nearby_panos.panoid should still be None')
        # publishing a panoid should set nearby_pano's panoid
        self.server.pub_panoid(metadata['location']['pano'])
        self.assertEqual(self.server.get_metadata(), None,
                         'metadata should still be none')
        self.assertEqual(self.server.nearby_panos.panoid, metadata['location']['pano'],
                         'nearby_panos.panoid should match metadata panoId')
        # now when handling metadata msg the metadata should be set
        self.server.handle_metadata_msg(msg)
        self.assertEqual(self.server.get_metadata(), metadata)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSVServer, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
