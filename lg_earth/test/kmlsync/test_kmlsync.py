#!/usr/bin/env python

PKG = 'lg_earth'
NAME = 'test_kmlsync'
KMLSYNC_HOST = '127.0.0.1'
KMLSYNC_PORT = 8765
KML_ENDPOINT = 'http://' + KMLSYNC_HOST + ':' + str(KMLSYNC_PORT)

WINDOW_SLUG = 'center'

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
from lg_earth import KmlUpdateHandler
from interactivespaces_msgs.msg import GenericMessage
from threading import Thread
from multiprocessing.pool import ThreadPool
from subprocess import Popen
QUERY_TOPIC = '/earth/query/tour'
SCENE_TOPIC = '/director/scene'
LPNODE = 'testing_kmlsync_node'
timeout_for_requests = 1

EMPTY_MESSAGE = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": []
    }
    """

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

class QueryTestSubscriber:
    def __init__(self, planet, tour):
        self.planet_pub = rospy.Publisher('/earth/planet', String, queue_size=1)
        self.planet_sub = rospy.Subscriber('/earth/query/planet', String, self.process_planet)
        self.playtour_sub = rospy.Subscriber('/earth/query/tour', String, self.process_tour)
        self.expected_planet = planet
        self.expected_tour = tour
        self.reset()

    def process_planet(self, data):
        #        if data.data == self.expected_planet:
        self.got_planet = True
        self.planet_pub.publish(self.expected_planet)
        sys.exit()

    def process_tour(self, data):
        if data.data == self.expected_tour and self.got_planet:
            self.got_planet = True

    def planet_received(self):
        return self.got_planet

    def tour_received(self):
        return self.got_tour

    def reset(self):
        self.got_planet = False
        self.got_tour = False


class TestKMLSync(unittest.TestCase):
    def setUp(self):
        self.session = requests.Session()
        self.test_planet = 'neptune'
        self.test_tour = 'lostinspace'
        self.query_test_subscriber = QueryTestSubscriber(self.test_planet, self.test_tour)
        rospy.Subscriber(QUERY_TOPIC, String, self._listen_query_string)
        self.wait_for_http()
        self.query_string = ''
        self._send_director_message(empty=True)

    def tearDown(self):
        self.session.close()

    def _listen_query_string(self, msg):
        self.query_string = msg.data

    def get_director_msg(self):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = DIRECTOR_MESSAGE
        return msg

    def get_empty_director_msg(self):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = EMPTY_MESSAGE
        return msg

    def get_request(self, url):
        r = self.session.get(url, timeout=timeout_for_requests, stream=False)
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
            rospy.resolve_name(LPNODE)), "%s is not up" % LPNODE)

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
        self._test_empty_cookie_string_when_no_state_is_set()

    def _test_empty_cookie_string_when_no_state_is_set(self):
        r = self.get_request(KML_ENDPOINT + '/network_link_update.kml?window_slug=' + WINDOW_SLUG)
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
        self._test_director_state()

    def _test_director_state(self):
        """
        Tests for the expected state when a director message has been sent.
        """
        r = self.get_request(KML_ENDPOINT + '/network_link_update.kml?window_slug=center')

        rospy.loginfo("r.content => '%s'" % escape(r.content))

        asset_urls = json.loads(DIRECTOR_MESSAGE)['windows'][0]['assets']

        expected_cookie = 'asset_slug=' + generate_cookie(asset_urls)
        expected_list_of_created_slugs = map(escape_asset_url, asset_urls)
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
        make a request for two commands, and see that it works
        """
        self.query_test_subscriber.reset()
        expected_status = 400
        bad1 = self.get_request(KML_ENDPOINT + "/query.html")
        bad2 = self.get_request(KML_ENDPOINT + "/query.html?query")
        bad3 = self.get_request(KML_ENDPOINT + "/query.html?query=")
        self.assertEqual(bad1.status_code, expected_status)
        self.assertEqual(bad2.status_code, expected_status)
        self.assertEqual(bad3.status_code, expected_status)

        expected_status = 200
        expected_string = "OK"

        #self.wait_for_pubsub()
        good1 = self.get_request(KML_ENDPOINT + "/query.html?query=playtour=myworldtour")
        rospy.sleep(1)
        good1_expected_string = "myworldtour"
        self.assertEqual(self.query_string, good1_expected_string)

        # NB! Google Earth won't play tours with spaces in the name, so don't
        # what this does here
        good2 = self.get_request(KML_ENDPOINT + "/query.html?query=playtour=My World Tour")
        rospy.sleep(1)
        good2_expected_string = "My World Tour"
        self.assertEqual(self.query_string, good2_expected_string)

        good3 = self.get_request(KML_ENDPOINT + "/query.html?query=planet=%s,playtour=%s" %
                (self.test_planet, self.test_tour))
        rospy.sleep(10)

        self.assertEqual(good1.status_code, expected_status)
        self.assertEqual(good2.status_code, expected_status)
        self.assertEqual(good1.content, expected_string)
        self.assertEqual(good2.content, expected_string)

    def test_8_send_request_before_state_change(self):
        """
        This test will make sure that requests sent before a statechange will
        get the proper return when the statechange happens before the request
        is returned
        """
        if timeout_for_requests <= 1:
            return  # not tesable with small timeout for requests
        t = Thread(target=self._sleep_and_send_director)
        t.start()
        self._test_director_state()
        t.join()

    def test_9_multiple_requests_before_state_change(self):
        """
        This tests when requests are made that require no state change
        sit on the dict while the state changes, and return with that
        new changed state.
        """
        if timeout_for_requests <= 1:
            return  # not tesable with small timeout for requests
        async_requests = []
        for i in range(5):
            pool = ThreadPool(processes=2)
            async_requests.append(pool.apply_async(self._test_director_state))
        self._send_director_message()
        for thread in async_requests:
            try:
                thread.get()
            except Exception:
                self.fail("Invalid director message retuned from queued request")

    def _send_director_message(self, empty=False):
        director_publisher = rospy.Publisher(SCENE_TOPIC, GenericMessage)
        rospy.sleep(1)
        msg = self.get_director_msg()
        if empty:
            msg = self.get_empty_director_msg()
        director_publisher.publish(msg)
        rospy.sleep(1)

    def _sleep_and_send_director(self):
        """
        Used to sleep then send a director message, helpful in our threads.
        """
        rospy.sleep(3)
        self._send_director_message()


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
    timeout_for_requests = rospy.get_param('~timeout_requests_session', 1)
    rostest.rosrun(PKG, NAME, TestKMLSync, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
