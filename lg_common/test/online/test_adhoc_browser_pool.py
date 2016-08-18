#!/usr/bin/env python

# test coverage for adhoc browser pool:
# - extensions:
#  - emit browser with extension - check if it got passed to --load-extension arg
#  - emit browser with 2 extensions - check above
#  - emit browser with 2 extensions and prelaoding - ros_window_ready should be loaded implicitly
# - command line argument passing:
#  - emit browser with custom command line args - verify that they've been added
# - persistence:
#   - emit two browsers without preloading - make service assert
#   - emit the same message again and verify that they havent been updated
#   - emit similar message but with a different slug - verify that they havent been touched
# - preloading:
#  - emit message with preloading - verify readiness message came, make service assert
#  - emit the same message with preloading again - verify readiness message came, make service assert and confirm that browsers were re-created
#  - emit the same message but with different slug - verify the same as above
#  - emit non-preloaded message with different slug and make usual asserts
#  - emit message with preloading - verify readiness message came, make service assert
#  - emit non-preloaded message with the same slug and make usual asserts
# - mix of preloading:
#  - emit two browsers - one with preloading - the other without preloading - verify readiness and service
#  - emit the same message again - non-preloaded browser needs to stay and preloaded browser needs to get re-created
#  - emit the same message again but with different slug - non-preloaded browser should stay, preloaded should get re-created
#  - emit non-preloaded message with different slug and make usual asserts
#  - emit two browsers - one with preloading - the other without preloading - verify readiness and service
#  - emit non-preloaded message with identical slug and make usual asserts
# - soft relaunch
#  - emit many browsers - preloaded and not preloaded - wait and make asserts
#  - make soft relaunch - verify they got killed
#  - emit many browsers - dont wait and make soft relaunch - all of them should get killed

PKG = 'lg_common'
NAME = 'test_adhoc_browser_pool'

import rospy
import unittest

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from lg_common.msg import WindowGeometry
from lg_common import ManagedWindow
from lg_common import AdhocBrowserDirectorBridge
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import extract_first_asset_from_director_message




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
        rospy.loginfo("zzz %s" % zero_id(AdhocBrowsers(browsers=[center_browser])))
        rospy.loginfo("zzz %s" % zero_id(self.mock_publisher_center.published_messages[0]))

        self.assertEqual(zero_id(AdhocBrowsers(browsers=[center_browser])), zero_id(self.mock_publisher_center.published_messages[0]))
        self.assertEqual(zero_id(AdhocBrowsers(browsers=[])), zero_id(self.mock_publisher_right.published_messages[0]))
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
        self.assertEqual(zero_id(AdhocBrowsers(browsers=[center_browser_1, center_browser_2, center_browser_3])), zero_id(self.mock_publisher_center.published_messages[0]))
        self.assertEqual(zero_id(AdhocBrowsers(browsers=[right_browser_4])), zero_id(self.mock_publisher_right.published_messages[0]))
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_center.published_messages[0]))
        self.assertEqual(AdhocBrowsers, type(self.mock_publisher_right.published_messages[0]))


def zero_id(d):
    for browser in d.browsers:
        browser.id = '0'
    return d

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowserDirectorBridge)
