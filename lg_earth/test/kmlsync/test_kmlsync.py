#!/usr/bin/env python

PKG = 'lg_earth'
NAME = 'test_kmlsync'
KMLSYNC_HOST = '127.0.0.1'
KMLSYNC_PORT = 8765
KML_ENDPOINT = 'http://' + KMLSYNC_HOST + ':' + str(KMLSYNC_PORT)

WINDOW_SLUG = 'test_window_slug'

import sys
import re
import rospy
import rostest
import time
import unittest
import requests
import xml.etree.ElementTree as ET

from xml.sax.saxutils import escape
from lg_common.helpers import escape_asset_url
from lg_common.helpers import write_log_to_file
from interactivespaces_msgs.msg import GenericMessage
from subprocess import Popen

PUBTOPIC = '/director/scene'
LPNODE = 'testing_kmlsync_node'

DIRECTOR_MESSAGE = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "earth",
                "assets": [
                        "http://lg-head:8060/media.kml",
                        "http://lg-head:8060/media/blah.kml",
                        "http://lg-head/zomgflolrlmao.kml"
                ],
                "height": 1080,
                "presentation_viewport": "center",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            },
            {
                "activity": "earth",
                "assets": [
                        "http://lg-head:8060/blah/right_one_content.kml"
                ],
                "height": 1080,
                "presentation_viewport": "right_one",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            }
            ]
    }
    """


class TestKMLSync(unittest.TestCase):
    def setUp(self):
        write_log_to_file("starting a test")
        self.wait_for_http()

    def _scene_listener(self, msg):
        write_log_to_file("Received message (inside TestKMLSync) %s" % msg)

    def get_director_msg(self):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = DIRECTOR_MESSAGE
        return msg

    def wait_for_pubsub(self):
        # wait at most 5 seconds for listenerpublisher to be registered
        timeout_t = time.time() + 5.0
        while not rostest.is_subscriber(
            rospy.resolve_name(PUBTOPIC),
            rospy.resolve_name(LPNODE)) and time.time() < timeout_t:
            time.sleep(0.1)

        self.assert_(rostest.is_subscriber(
            rospy.resolve_name(PUBTOPIC),
            rospy.resolve_name(LPNODE)), "%s is not up"%LPNODE)

    def wait_for_http(self):
        # TODO: implement this
        rospy.sleep(1.0)

    def test_1_master_kml_200(self):
        r = requests.get(KML_ENDPOINT + '/master.kml')
        result = r.status_code
        expected = 200
        self.assertEqual(result, expected)

    def test_2_master_kml_content(self):
        r = requests.get(KML_ENDPOINT + '/master.kml')
        result = ET.fromstring(r.content).find('.//{http://www.opengis.net/kml/2.2}Document').attrib['id']
        expected = 'master'
        self.assertEqual(result, expected)

    def test_3_network_link_update_kml_without_params(self):
        r = requests.get(KML_ENDPOINT + '/network_link_update.kml')
        result = r.status_code
        expected = 400
        self.assertEqual(result, expected)

    def test_4_network_link_update_cookie_string_is_initially_empty(self):
        r = requests.get(KML_ENDPOINT + '/network_link_update.kml?window_slug=' + WINDOW_SLUG)
        result = get_cookie_string(r.content)
        expected = ''
        self.assertEqual(result, expected)

    def test_5_cookie_string_from_director(self):
        """
         - send director message
         - assert assets and cookie string
         - make GET request to assert:
            - cookie string
            - CREATE/DELETE sections
        """

        self.wait_for_pubsub()
        self.sub = rospy.Subscriber('/director/scene', GenericMessage, self._scene_listener)
        director_publisher = rospy.Publisher(PUBTOPIC, GenericMessage)
        rospy.sleep(1)
        msg = self.get_director_msg()
        director_publisher.publish(msg)
        write_log_to_file("Published a message on topic: %s with %s" % (msg, director_publisher))
        rospy.sleep(1)

        r = requests.get(KML_ENDPOINT + '/network_link_update.kml?window_slug=center')

        rospy.loginfo("r.content => '%s'" % escape(r.content))

        expected_cookie = ''
        expected_number_of_create_elements = 3
        expected_list_of_slugs = map( escape_asset_url, ["http://lg-head:8060/media.kml", "http://lg-head:8060/media/blah.kml", "http://lg-head/zomgflolrlmao.kml"])
        expected_number_of_delete_elements = 0

def get_cookie_string(s):
    return re.search('\\<\\!\\[CDATA\\[(.*)\\]\\]\\>', s, re.M).groups()[0]

if __name__ == '__main__':
    rospy.init_node('test_director')
    rostest.rosrun(PKG, NAME, TestKMLSync, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
