#!/usr/bin/env python

PKG = 'lg_common'
NAME = 'test_adhoc_browser'

import rospy
import unittest

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from lg_common import AdhocBrowserPool
from lg_common import AdhocBrowserDirectorBridge
from std_msgs.msg import String


class MockSubscriber:
    def __init__(self):
        self.messages = []

    def reinitialize(self):
        self.messages = []

    def record_message(self, message):
        self.messages.append(message)


class TestAdhocBrowserPool(unittest.TestCase):
    def setUp(self):
        """
         test coverage for adhoc browser pool:
         - extensions:
          - emit browser with extension - check if it got passed to --load-extension arg
          - emit browser with 2 extensions - check above
          - emit browser with 2 extensions and prelaoding - ros_window_ready should be loaded implicitly
         - command line argument passing:
          - emit browser with custom command line args - verify that they've been added
         - persistence:
           - emit two browsers without preloading - make service assert
           - emit the same message again and verify that they havent been updated
           - emit similar message but with a different slug - verify that they havent been touched
         - preloading:
          - emit message with preloading - verify readiness message came, make service assert
          - emit the same message with preloading again - verify readiness message came, make service assert and confirm that browsers were re-created
          - emit the same message but with different slug - verify the same as above
          - emit non-preloaded message with different slug and make usual asserts
          - emit message with preloading - verify readiness message came, make service assert
          - emit non-preloaded message with the same slug and make usual asserts
         - mix of preloading:
          - emit two browsers - one with preloading - the other without preloading - verify readiness and service
          - emit the same message again - non-preloaded browser needs to stay and preloaded browser needs to get re-created
          - emit the same message again but with different slug - non-preloaded browser should stay, preloaded should get re-created
          - emit non-preloaded message with different slug and make usual asserts
          - emit two browsers - one with preloading - the other without preloading - verify readiness and service
          - emit non-preloaded message with identical slug and make usual asserts
         - soft relaunch
          - emit many browsers - preloaded and not preloaded - wait and make asserts
          - make soft relaunch - verify they got killed
          - emit many browsers - dont wait and make soft relaunch - all of them should get killed

         assert types:
          - /browser_service/center AdhocBrowsers
          - /browser_service/left AdhocBrowsers
          - /browser_service/right AdhocBrowsers
          - /browser_service/browsers AdhocBrowsers
          - /director/ready Ready
          - /director/window/ready String
          - last but not least: rosservice for adhoc browser pool
        """

        self.subscribers = []
        self.browser_service_mock_center = MockSubscriber()
        self.browser_service_mock_left = MockSubscriber()
        self.browser_service_mock_right = MockSubscriber()
        self.browser_service_mock_common = MockSubscriber()
        self.director_window_ready_mock = MockSubscriber()
        self.director_ready_mock = MockSubscriber()

        self.subscribers.append(self.browser_service_mock_center)
        self.subscribers.append(self.browser_service_mock_left)
        self.subscribers.append(self.browser_service_mock_right)
        self.subscribers.append(self.browser_service_mock_common)
        self.subscribers.append(self.director_window_ready_mock)
        self.subscribers.append(self.director_ready_mock)

        rospy.Subscriber('/browser_service/center',
                self.browser_service_mock_center.record_message,
                AdhocBrowsers)
        rospy.Subscriber('/browser_service/left',
                self.browser_service_mock_left.record_message,
                AdhocBrowsers)
        rospy.Subscriber('/browser_service/right',
                self.browser_service_mock_right.record_message,
                AdhocBrowsers)
        rospy.Subscriber('/browser_service/browsers',
                self.browser_service_mock_common.record_message,
                AdhocBrowsers)
        rospy.Subscriber('/director/ready',
                self.director_ready_mock.record_message,
                Ready)
        rospy.Subscriber('/director/window/ready',
                self.browser_service_mock_common.record_message,
                String)
        # extensions messages
        self.single_browser_with_extension = None
        self.single_browser_with_two_extensions = None
        self.single_browser_with_two_extensions_and_preloading = None
        # cmdline args messages
        self.single_browser_with_cmdline_args = None
        # persistence
        self.two_browsers_no_preload = None
        self.two_browsers_no_preload_alt_slug = None
        # preloading
        self.two_browsers_with_preloading = None
        self.two_browsers_with_preloading_alt_slug = None
        # preloading mix
        self.two_browsers_with_preloading_mix = None
        self.two_browsers_with_preloading_mix_alt_slug = None
        # soft relaunch
        self.soft_relaunch_message = String(data='media')
        # director_bridge_mocks
        director_pool = None
        # director scene publisher
        director_publisher = rospy.Publisher()

    def reinitialize_mock_subscribers(self):
        [subscriber.reinitialize() for subscriber in self.subscribers]

    def test_1_tests_are_working(self):
        """
        """
        self.reinitialize_mock_subscribers()
        self.assertEqual(1, 1)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowser)
