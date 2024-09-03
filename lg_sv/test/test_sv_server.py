#!/usr/bin/env python3

PKG = 'lg_sv'
NAME = 'test_sv_server'
LAT = '43.726319'
LON = '12.636377'

TILT_MAX = 80
TILT_MIN = -80
NAV_SENSITIVITY = 1
NAV_INTERVAL = .1
IDLE_TIL_SNAP = 5.0

import sys
import json
import rospy
import rostest
import unittest
from lg_sv import StreetviewUtils, PanoViewerServer, NearbyStreetviewPanos
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import String
from lg_common.helpers import get_first_asset_from_activity, load_director_message
from interactivespaces_msgs.msg import GenericMessage


class MockDirectorToPanoidPub:
    def __init__(self):
        self.data = []

    def publish(self, data):
        panoid = json.loads(data.message).get('windows', [{}])[0].get('activity_config', {}).get('panoid', '')
        self.data.append(panoid)


class MockPublisher:
    def __init__(self):
        self.data = []

    def publish(self, data):
        self.data.append(data)


class TestSVServer(unittest.TestCase):
    def setUp(self):
        self.location_pub = MockPublisher()
        self.old_pano_pub = MockPublisher()
        self.pano_pub = MockDirectorToPanoidPub()
        self.pov_pub = MockPublisher()
        self.server = PanoViewerServer(
            self.location_pub, self.old_pano_pub, self.pov_pub, TILT_MIN,
            TILT_MAX, NAV_SENSITIVITY, NAV_INTERVAL, IDLE_TIL_SNAP,
            nearby_panos=NearbyStreetviewPanos(), director_pub=self.pano_pub)

    def tearDown(self):
        pass

    def check_init_state(self):
        ss = self.server
        self.assertFalse(ss.button_down)
        self.assertEqual(0, ss.last_nav_msg_t)
        self.assertEqual(0, ss.time_since_last_nav_msg)
        self.assertEqual(0, ss.move_forward)
        self.assertEqual(0, ss.move_backward)
        self.assertFalse(ss.last_metadata)
        self.assertEqual(0, ss.location.x)
        self.assertEqual(0, ss.location.y)
        self.assertEqual(0, ss.location.theta)
        self.assertEqual(0, ss.pov.x)
        self.assertEqual(0, ss.pov.y)
        self.assertEqual(0, ss.pov.z)
        #self.assertEqual(INITIAL_ZOOM, ss.pov.w)
        self.assertFalse(ss.panoid)
        self.assertEqual(0, ss.last_twist_msg.linear.x)
        self.assertEqual(0, ss.last_twist_msg.linear.y)
        self.assertEqual(0, ss.last_twist_msg.linear.z)
        self.assertEqual(0, ss.last_twist_msg.angular.x)
        self.assertEqual(0, ss.last_twist_msg.angular.y)
        self.assertEqual(0, ss.last_twist_msg.angular.z)

    def check_soft_relaunch(self):
        ss = self.server
        ss.handle_soft_relaunch()
        self.check_init_state()

    def test_sv_utils(self):
        return
        response = StreetviewUtils.get_panoid_from_lat_lon(LAT, LON)
        self.assertIsInstance(response, str)

    def test_sv_utils_metadata(self):
        return
        response = StreetviewUtils.get_metadata_from_lat_lon(LAT, LON)
        self.assertIsInstance(response, dict)
        self.assertTrue('Location' in response)
        self.assertTrue('panoId' in response['Location'])
        webapp_metadata = StreetviewUtils.translate_server_metadata_to_client_form(response)
        self.assertEqual(response['Location']['panoId'], webapp_metadata['location']['pano'],
                         'webapp metadata does not match the google version')

    def test_init_state(self):
        self.check_init_state()

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

        self.check_soft_relaunch()

    def test_2_pov_pub(self):
        # test initial state
        self.assertEqual(len(self.pov_pub.data), 0,
                         'pov_pub data should not have anything in it')
        q = Quaternion()
        # w starts as a non-zero value for zoom supported clients, it's
        # not worth it to hard code that value in here, so just grab it
        # from the current pov
        q.w = self.server.pov.w
        self.assertEqual(self.server.pov, q,
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

        self.check_soft_relaunch()

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

        self.check_soft_relaunch()

    def test_4_handle_location_msg(self):
        return
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

        self.check_soft_relaunch()

    def test_5_handle_metadata_msg(self):
        return
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

        self.check_soft_relaunch()

    def test_6_pano_change(self):
        return
        # set up initial state
        # grab the metadata for the pano we will be working with
        metadata = StreetviewUtils.get_metadata_from_lat_lon(LAT, LON)
        metadata = StreetviewUtils.translate_server_metadata_to_client_form(metadata)
        # publish the pano so the server.panoid and nearby_panos.panoid get set
        self.server.pub_panoid(metadata['location']['pano'])
        msg = String(json.dumps(metadata))
        # allow the nearby_pano member to set the metadata
        self.server.handle_metadata_msg(msg)
        # check that links has members, otherwise there will be no pano change
        if len(metadata['links']) == 0:
            return
        pov = Quaternion()
        pov_z = pov.z
        # store the return of find_closest to check to test
        new_pano = self.server.nearby_panos.find_closest(metadata['location']['pano'], pov_z)
        self.assertNotEqual(new_pano, None,
                            'there should be a new pano since there were links')
        self.assertNotEqual(new_pano, metadata['location']['pano'],
                            'the new pano should not be the same as the old pano')

        # publish a pov since it is needed to move
        self.server.pub_pov(pov)
        self.server._move_forward()
        # check that after moving forward the server's panoid matches the expected
        self.assertEqual(self.server.panoid, new_pano,
                         'the new pano should match the one returned from find_closest')
        self.assertEqual(self.server.get_metadata(), None,
                         'metadata should be none since the new metadata has not been published yet')

        self.check_soft_relaunch()

    def test_7_director_translation(self):
        panoid = "1234asdf4321fdsa"
        director_msg = GenericMessage()
        director_msg.type = 'json'
        director_msg.message = """
                {
                      "description":"",
                      "duration":30,
                      "name":"Browser service test",
                      "resource_uri":"/director_api/scene/browser-service-test/",
                      "slug":"browser-service-test",
                      "windows":[
                        {
                        "activity":"streetview",
                        "assets":["%s", "foo", "bar"],
                        "height":600,
                        "presentation_viewport":"center",
                        "width":800,
                        "x_coord":100,
                        "y_coord":100
                        }
                        ]
                    }
        """ % panoid

        # get existing asset
        asset = get_first_asset_from_activity(
            load_director_message(director_msg), "streetview")
        self.assertEqual(asset, panoid, 'Invalid asset returned')
        # get non existing asset
        asset = get_first_asset_from_activity(
            load_director_message(director_msg), "panoview")
        self.assertEqual(asset, None, 'No asset should have been returned')

        self.check_soft_relaunch()


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSVServer, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
