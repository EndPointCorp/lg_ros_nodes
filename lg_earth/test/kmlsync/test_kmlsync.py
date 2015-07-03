#!/usr/bin/env python

PKG = 'lg_earth'
NAME = 'test_kmlsync'
KMLSYNC_HOST = '127.0.0.1'
KMLSYNC_PORT = 8765
KML_ENDPOINT = 'http://' + KMLSYNC_HOST + ':' + str(KMLSYNC_PORT)

WINDOW_SLUG = 'test_window_slug'

import sys
import re
import time
import json
import rospy
import rostest
import unittest
import requests
import xml.etree.ElementTree as ET
from std_msgs.msg import String
from xml.sax.saxutils import escape
from lg_common.helpers import escape_asset_url, generate_cookie
from lg_common.helpers import write_log_to_file
from interactivespaces_msgs.msg import GenericMessage
from subprocess import Popen

PUBTOPIC = '/earth/query/tour'
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
        self.session = requests.Session()
        self.wait_for_http()
        self.query_string = ''

    def _scene_listener(self, msg):
        write_log_to_file("Receiveu message (inside TestKMLSync) %s" % msg)

    def _listen_query_string(self, msg):
        self.query_string = msg.data
        write_log_to_file("Received query string %s" % self.query_string)

    def get_director_msg(self):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = DIRECTOR_MESSAGE
        return msg

    def get_request(self, url):
        r = self.session.get(url)
        return r

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
        rospy.sleep(3.0)

    def test_1_master_kml_200(self):
        r = self.get_request(KML_ENDPOINT + '/master.kml')
        result = r.status_code
        expected = 200
        self.assertEqual(result, expected)

    def test_2_master_kml_content(self):
        r = self.get_request(KML_ENDPOINT + '/master.kml')
        result = ET.fromstring(r.content).find('.//{http://www.opengis.net/kml/2.2}Document').attrib['id']
        expected = 'master'
        self.assertEqual(result, expected)

    def test_3_network_link_update_kml_without_params(self):
        r = self.get_request(KML_ENDPOINT + '/network_link_update.kml')
        result = r.status_code
        expected = 400
        self.assertEqual(result, expected)

    def test_4_network_link_update_cookie_string_is_initially_empty(self):
        r = self.get_request(KML_ENDPOINT + '/network_link_update.kml?window_slug=' + WINDOW_SLUG)
        write_log_to_file("r.content => '%s'" % r.content)
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
        When there no assets to be loaded or unloaded, the KML should look like this:

            <?xml version="1.0" encoding="UTF-8"?>
            <kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
            <NetworkLinkControl>
            <minRefreshPeriod>1</minRefreshPeriod>
            <maxSessionLength>-1</maxSessionLength>
            <cookie><![CDATA[]]></cookie>
            <Update>
                <targetHref>http://localhost:9001/master.kml</targetHref>
            </Update>
            </NetworkLinkControl>
            </kml>
        """

        self._send_director_message()

        r = self.get_request(KML_ENDPOINT + '/network_link_update.kml?window_slug=center')

        rospy.loginfo("r.content => '%s'" % escape(r.content))

        asset_urls = json.loads(DIRECTOR_MESSAGE)['windows'][0]['assets']

        expected_cookie = 'asset_slug=' + generate_cookie(asset_urls)
        expected_list_of_created_slugs = map( escape_asset_url, asset_urls)
        expected_list_of_deleted_slugs = []

        # start testing...
        self.assertEqual(expected_cookie, get_cookie_string(r.content))
        self.assertEqual(sorted(expected_list_of_created_slugs), sorted(get_created_elements(r.content)))
        self.assertEqual(expected_list_of_deleted_slugs, get_deleted_elements(r.content))

    def test_6_asset_state_in_url(self):
        self._send_director_message()

        assets = json.loads(DIRECTOR_MESSAGE)['windows'][0]['assets']
        delete_slug = 'http___foo_bar_kml'
        cookie = 'asset_slug=' + generate_cookie([assets[0], delete_slug])
        r = self.get_request(KML_ENDPOINT + '/network_link_update.kml?window_slug=center&%s' % cookie)

        expected_list_of_created_slugs = map(escape_asset_url, assets[1:])
        expected_list_of_deleted_slugs = [delete_slug]
        self.assertEqual(sorted(expected_list_of_created_slugs), sorted(get_created_elements(r.content)))
        self.assertEqual(expected_list_of_deleted_slugs, get_deleted_elements(r.content))

    def test_7_queryfile_message(self):
        """
        make a bad get request to get html and assert for 400
        make a legit get request to get 'OK' and status_code 200 and assert for the message that was sent
        """
        rospy.Subscriber('/earth/query/tour', String, self._listen_query_string)
        expected_status = 400
        bad1 = self.get_request(KML_ENDPOINT+"/query.html")
        bad2 = self.get_request(KML_ENDPOINT+"/query.html?query")
        bad3 = self.get_request(KML_ENDPOINT+"/query.html?query=")
        self.assertEqual(bad1.status_code, expected_status)
        self.assertEqual(bad2.status_code, expected_status)
        self.assertEqual(bad3.status_code, expected_status)

        expected_status = 200
        expected_string = "OK"

        #self.wait_for_pubsub()
        good1 = self.get_request(KML_ENDPOINT+"/query.html?query=tour=myworldtour")
        good1_expected_string = "myworldtour"
        self.assertEqual(self.query_string, good1_expected_string)

        good2 = self.get_request(KML_ENDPOINT+"/query.html?query=tour=My World Tour")
        good2_expected_string = "My World Tour"
        self.assertEqual(self.query_string, good2_expected_string)

        self.assertEqual(good1.status_code, expected_status)
        self.assertEqual(good2.status_code, expected_status)
        self.assertEqual(good1.content, expected_string)
        self.assertEqual(good2.content, expected_string)

    def _send_director_message(self):
        director_publisher = rospy.Publisher(PUBTOPIC, GenericMessage)
        rospy.sleep(1)
        msg = self.get_director_msg()
        director_publisher.publish(msg)
        write_log_to_file("Published a message on topic: %s " % (director_publisher))
        rospy.sleep(1)


def get_cookie_string(s):
    return re.search('\\<\\!\\[CDATA\\[(.*)\\]\\]\\>', s, re.M).groups()[0]

def get_created_elements(x):
    try:
        tmp = ET.fromstring(x).find('.//{http://www.opengis.net/kml/2.2}Create').findall('.//{http://www.opengis.net/kml/2.2}name')
        return [elem.text for elem in tmp]
    except AttributeError:
        return []

def get_deleted_elements(x):
    try:
        return [elem.attrib['targetId'] for elem in ET.fromstring(x).find('.//{http://www.opengis.net/kml/2.2}Delete').getchildren()]
    except AttributeError:
        return []

if __name__ == '__main__':
    rospy.init_node('test_director')
    rostest.rosrun(PKG, NAME, TestKMLSync, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
