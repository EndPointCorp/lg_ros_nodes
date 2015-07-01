#!/usr/bin/env python

PKG = 'lg_earth'
NAME = 'test_kmlsync'
KMLSYNC_HOST = '127.0.0.1'
KMLSYNC_PORT = 8765
KML_ENDPOINT = 'http://' + KMLSYNC_HOST + ':' + str(KMLSYNC_PORT)
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

WINDOW_SLUG = 'test_window_slug'

import re
import rospy
import unittest
import requests
import xml.etree.ElementTree as ET

from lg_common.helpers import escape_asset_url
from interactivespaces_msgs.msg import GenericMessage


class DirectorPublisher:
    def __init__(self):
        self.director_publisher = rospy.Publisher('/director/scene', GenericMessage, queue_size=10, latch=True)
        rospy.loginfo("Initliazed %s" % self.__class__)

    def publish(self):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = DIRECTOR_MESSAGE
        rospy.loginfo("About to publish message %s" % msg.message)
        self.director_publisher.publish(msg)
        rospy.loginfo("Pubished the message from %s" % self.__class__)


class TestKMLSync(unittest.TestCase):
    def setUp(self):
        rospy.loginfo("Starting test for KMLSync and TestDirectorPublisher")
        self.director_publisher = rospy.Publisher('/director/scene', GenericMessage, queue_size=10, latch=True)

    def publish(self):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = DIRECTOR_MESSAGE
        rospy.loginfo("About to publish message %s" % msg.message)
        self.director_publisher.publish(msg)
        rospy.loginfo("Pubished the message from %s" % self.__class__)


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

        r = requests.get(KML_ENDPOINT + '/network_link_update.kml?window_slug=center')
        rospy.loginfo("response from networklink XXX: %s" % r.content)
        expected_cookie = ''
        expected_number_of_create_elements = 3
        expected_list_of_slugs = map( escape_asset_url, ["http://lg-head:8060/media.kml", "http://lg-head:8060/media/blah.kml", "http://lg-head/zomgflolrlmao.kml"])
        expected_number_of_delete_elements = 0

def get_cookie_string(s):
    return re.search('\\<\\!\\[CDATA\\[(.*)\\]\\]\\>', s, re.M).groups()[0]

if __name__ == '__main__':
    import rostest
    rospy.sleep(1)
    rostest.rosrun(PKG, NAME, TestKMLSync)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
