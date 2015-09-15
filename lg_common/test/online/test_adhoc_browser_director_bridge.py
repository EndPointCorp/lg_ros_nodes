#!/usr/bin/env python

PKG = 'lg_common'
NAME = 'test_adhoc_browser_director_bridge'

import rospy
import unittest

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from lg_common.msg import WindowGeometry
from lg_common import ManagedWindow
from lg_common import AdhocBrowserDirectorBridge
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import extract_first_asset_from_director_message

DIRECTOR_MESSAGE_BOGUS = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "bogus",
                "assets": [
                    "whatever"
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
                "http://www.doogal.co.uk/LondonTubeLinesKML.php"
                ],
                "height": 1080,
                "presentation_viewport": "right",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            },
            {
                "activity": "earth",
                "assets": [
                    "http://www.doogal.co.uk/LondonTubeLinesKML.php"
                ],
                "height": 1080,
                "presentation_viewport": "left",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            }
            ]
    }
    """

DIRECTOR_MESSAGE_CENTER = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "browser",
                "assets": [
                    "http://www.lol.zomg"
                ],
                "height": 800,
                "presentation_viewport": "center",
                "width": 600,
                "x_coord": 10,
                "y_coord": 10
            },
            {
                "activity": "earth",
                "assets": [
                "http://www.doogal.co.uk/LondonTubeLinesKML.php"
                ],
                "height": 1080,
                "presentation_viewport": "left_one",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            },
            {
                "activity": "earth",
                "assets": [
                    "http://www.doogal.co.uk/LondonTubeLinesKML.php"
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

DIRECTOR_MESSAGE_CENTER_3 = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "browser",
                "assets": [
                    "http://www.lol.zomg"
                ],
                "height": 800,
                "presentation_viewport": "center",
                "width": 600,
                "x_coord": 10,
                "y_coord": 10
            },
            {
                "activity": "browser",
                "assets": [
                    "http://www.lol2.zomg"
                ],
                "height": 200,
                "presentation_viewport": "center",
                "width": 300,
                "x_coord": 400,
                "y_coord": 200
            },
            {
                "activity": "browser",
                "assets": [
                    "http://www.lol3.zomg"
                ],
                "height": 11,
                "presentation_viewport": "center",
                "width": 888,
                "x_coord": 10,
                "y_coord": 10
            },
            {
                "activity": "browser",
                "assets": [
                    "http://www.lol4.zomg"
                ],
                "height": 100,
                "presentation_viewport": "right",
                "width": 100,
                "x_coord": 100,
                "y_coord": 100
            },
            {
                "activity": "earth",
                "assets": [
                "http://www.doogal.co.uk/LondonTubeLinesKML.php"
                ],
                "height": 1080,
                "presentation_viewport": "left_one",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            },
            {
                "activity": "earth",
                "assets": [
                    "http://www.doogal.co.uk/LondonTubeLinesKML.php"
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


class MockBrowserPoolPublisher:
    def __init__(self):
        self.published_messages = []

    def publish(self, adhoc_browsers):
        rospy.logdebug("Publishing adhoc browsers: %s" % adhoc_browsers)
        self.published_messages.append(adhoc_browsers)
        rospy.logdebug("After publishing, self.published_messages = %s" % self.published_messages)


class TestAdhocBrowserDirectorBridge(unittest.TestCase):
    def setUp(self):
        """
        - check whether bridge translates json with director message to proper message for 2 viewports
        - two messages should be sent:
            - bogus message - no director should catch that
            - message addressed to center viewport
        """
        self.message_bogus = GenericMessage()
        self.message_bogus.type = 'json'
        self.message_bogus.message = DIRECTOR_MESSAGE_BOGUS

        self.message_center = GenericMessage()
        self.message_center.type = 'json'
        self.message_center.message = DIRECTOR_MESSAGE_CENTER

        self.message_3 = GenericMessage()
        self.message_3.type = 'json'
        self.message_3.message = DIRECTOR_MESSAGE_CENTER_3

        self.mock_publisher_center = MockBrowserPoolPublisher()
        self.mock_publisher_right = MockBrowserPoolPublisher()
        self.bridge_center = AdhocBrowserDirectorBridge(self.mock_publisher_center, 'center')
        self.bridge_right = AdhocBrowserDirectorBridge(self.mock_publisher_right, 'right')

    def test_1_bogus_director_message_is_ignored(self):
        """
        Send unrelated director message and check if empty list of browsers is published
        """
        self.bridge_center.translate_director(self.message_bogus)
        self.bridge_right.translate_director(self.message_bogus)

        self.assertEqual(1, len(self.mock_publisher_center.published_messages))
        self.assertEqual(1, len(self.mock_publisher_right.published_messages))
        self.assertEqual(AdhocBrowsers(browsers=[]), self.mock_publisher_center.published_messages[0])
        self.assertEqual(AdhocBrowsers(browsers=[]), self.mock_publisher_right.published_messages[0])
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_center.published_messages[0]))
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_right.published_messages[0]))

    def test_2_center_message(self):
        """
        Send message containing one asset directed at 'center' viewport
        assert for:
        - one browser on 'center' viewport and check url + geometry (with viewport offset)
        - empty browsers list on the other viewport
        """
        self.bridge_center.translate_director(self.message_center)
        self.bridge_right.translate_director(self.message_center)

        self.assertEqual(1, len(self.mock_publisher_center.published_messages))
        self.assertEqual(1, len(self.mock_publisher_right.published_messages))

        center_browser = AdhocBrowser(id='adhoc_browser_center_0',
                                      geometry=WindowGeometry(x=10,
                                                              y=10,
                                                              width=600,
                                                              height=800),
                                      url='http://www.lol.zomg')

        rospy.loginfo("published adhoc browser => %s" % self.mock_publisher_center.published_messages[0])
        rospy.loginfo("asserted adhoc browser => %s" % AdhocBrowsers(browsers=center_browser))

        self.assertEqual(AdhocBrowsers(browsers=[center_browser]), self.mock_publisher_center.published_messages[0])
        self.assertEqual(AdhocBrowsers(browsers=[]), self.mock_publisher_right.published_messages[0])
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_center.published_messages[0]))
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_right.published_messages[0]))

    def test_3_multi_browser_message(self):
        """
        Send message containing one asset directed at 'center' viewport
        assert for:
        - one browser on 'center' viewport and check url + geometry (with viewport offset)
        - empty browsers list on the other viewport
        """
        self.bridge_center.translate_director(self.message_3)
        self.bridge_right.translate_director(self.message_3)
        self.assertEqual(1, len(self.mock_publisher_center.published_messages))
        self.assertEqual(1, len(self.mock_publisher_right.published_messages))
        center_browser_1 = AdhocBrowser(id='adhoc_browser_center_0',
                                        geometry=WindowGeometry(x=10,
                                                                y=10,
                                                                width=600,
                                                                height=800),
                                        url='http://www.lol.zomg')

        center_browser_2 = AdhocBrowser(id='adhoc_browser_center_1',
                                        geometry=WindowGeometry(x=400,
                                                                y=200,
                                                                width=300,
                                                                height=200),
                                        url='http://www.lol2.zomg')

        center_browser_3 = AdhocBrowser(id='adhoc_browser_center_2',
                                        geometry=WindowGeometry(x=10,
                                                                y=10,
                                                                width=888,
                                                                height=11),
                                        url='http://www.lol3.zomg')

        right_browser_4 = AdhocBrowser(id='adhoc_browser_right_0',
                                       geometry=WindowGeometry(x=100,
                                                               y=100,
                                                               width=100,
                                                               height=100),
                                       url='http://www.lol4.zomg')

        rospy.loginfo("published adhoc browser => %s" % self.mock_publisher_center.published_messages[0])
        rospy.loginfo("asserted adhoc browser => %s" % AdhocBrowsers(browsers=[center_browser_1, center_browser_2, center_browser_3]))
        self.assertEqual(AdhocBrowsers(browsers=[center_browser_1, center_browser_2, center_browser_3]), self.mock_publisher_center.published_messages[0])
        self.assertEqual(AdhocBrowsers(browsers=[right_browser_4]), self.mock_publisher_right.published_messages[0])
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_center.published_messages[0]))
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_right.published_messages[0]))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowserDirectorBridge)
