#!/usr/bin/env python

PKG = 'lg_common'
NAME = 'test_adhoc_browser_url_monitor'

import rospy
import unittest
import tempfile
import os
import json

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from lg_common import InteractiveSpacesMessagesFactory
from lg_common.helpers import write_log_to_file
from lg_common.srv import BrowserPool


class MockSubscriber(object):
    def __init__(self, topic_name=None):
        self.topic_name = topic_name
        self.messages = []

    def __repr__(self):
        return "%s's MockSubscriber" % self.topic_name

    def reinitialize(self):
        self.messages = []

    def record_message(self, message):
        self.messages.append(message)


class TestAdhocBrowser(unittest.TestCase):
    def setUp(self):
        self.loading_grace_time = 20
        self.message_emission_grace_time = 3
        self.message_factory = InteractiveSpacesMessagesFactory()
        self.subscribers = []
        self.browser_service_mock_center = MockSubscriber(topic_name='/browser_service/center')

        self.subscribers.append(self.browser_service_mock_center)

        rospy.Subscriber(
            '/browser_service/center',
            AdhocBrowsers,
            self.browser_service_mock_center.record_message
        )
        rospy.init_node("test_adhoc_browser_url_monitor", anonymous=True)
        rospy.sleep(3)

        # director scene publisher
        self.director_publisher = rospy.Publisher(
            '/director/scene',
            GenericMessage,
            queue_size=3
        )

    def tearDown(self):
        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)

    def reinitialize_mock_subscribers(self):
        [subscriber.reinitialize() for subscriber in self.subscribers]

    def print_mock_subscribers(self):
        write_log_to_file("============")
        [write_log_to_file("%s's messages: %s" % (subscriber, subscriber.messages)) for subscriber in self.subscribers]
        write_log_to_file("============")

    def test_1_tests_are_working(self):
        """
        """
        self.reinitialize_mock_subscribers()
        self.assertEqual(1, 1)

    def test_2_chrome_extension_initialization(self):
        """
        emit browser with extension - check if it got passed to --load-extension arg
        """
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_allowed_urls_msg'))
        rospy.sleep(self.message_emission_grace_time)

        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].allowed_urls, ['google.com', 'endpoint.com'])

    def test_3_extension_gets_passed_to_chrome_cmdline(self):
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_allowed_urls_msg'))
        rospy.sleep(self.message_emission_grace_time)

        rospy.wait_for_service('/browser_service/center')
        center_service = rospy.ServiceProxy('/browser_service/center', BrowserPool)
        browsers_on_center = center_service().state

        try:
            browsers_on_center = json.loads(browsers_on_center)
            json_is_valid = True
        except ValueError:
            browsers_on_center = {}
            json_is_valid = False

        self.assertEqual(json_is_valid, True)
        self.assertEqual(len(browsers_on_center), 1)
        self.assertEqual('monitor_page_urls' in browsers_on_center.items()[0][1]['extensions'][0], True)
        self.assertEqual('allowed_urls=google.com' in browsers_on_center.items()[0][1]['url'], True)
        self.assertEqual('allowed_urls=endpoint.com' in browsers_on_center.items()[0][1]['url'], True)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowser)
