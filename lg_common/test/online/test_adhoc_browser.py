#!/usr/bin/env python

PKG = 'lg_common'
NAME = 'test_adhoc_browser'

import rospy
import unittest
import tempfile
import os
import json

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from lg_common.msg import Ready
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
        """
         assert types used in below tests:
          - /browser_service/center AdhocBrowsers
          - /browser_service/left AdhocBrowsers
          - /browser_service/right AdhocBrowsers
          - /browser_service/browsers AdhocBrowsers
          - /director/ready Ready
          - /director/window/ready String
          - last but not least: rosservice for adhoc browser pool
        """

        self.preloading_grace_time = 20
        self.message_emission_grace_time = 3
        self.message_factory = InteractiveSpacesMessagesFactory()
        self.subscribers = []
        self.browser_service_mock_center = MockSubscriber(topic_name='/browser_service/center')
        self.browser_service_mock_left = MockSubscriber(topic_name='/browser_service/left')
        self.browser_service_mock_right = MockSubscriber(topic_name='/browser_service/right')
        self.browser_service_mock_common = MockSubscriber(topic_name='/browser_service/browsers')
        self.director_window_ready_mock = MockSubscriber(topic_name='/director/window/ready')
        self.director_ready_mock = MockSubscriber(topic_name='/director/ready')
        self.director_scene_mock = MockSubscriber(topic_name='/director/scene')
        self.common_mock = MockSubscriber(topic_name='all_topics')  # subscriber for all messages

        self.subscribers.append(self.browser_service_mock_center)
        self.subscribers.append(self.browser_service_mock_left)
        self.subscribers.append(self.browser_service_mock_right)
        self.subscribers.append(self.browser_service_mock_common)
        self.subscribers.append(self.director_window_ready_mock)
        self.subscribers.append(self.director_ready_mock)
        self.subscribers.append(self.director_scene_mock)
        self.subscribers.append(self.common_mock)  # common channel for all msgs

        rospy.Subscriber(
            '/browser_service/center',
            AdhocBrowsers,
            self.browser_service_mock_center.record_message
        )
        rospy.Subscriber(
            '/browser_service/left',
            AdhocBrowsers,
            self.browser_service_mock_left.record_message
        )
        rospy.Subscriber(
            '/browser_service/right',
            AdhocBrowsers,
            self.browser_service_mock_right.record_message
        )
        rospy.Subscriber(
            '/browser_service/browsers',
            AdhocBrowsers,
            self.browser_service_mock_common.record_message
        )
        rospy.Subscriber(
            '/director/ready',
            Ready,
            self.director_ready_mock.record_message
        )
        rospy.Subscriber(
            '/director/window/ready',
            String,
            self.director_window_ready_mock.record_message
        )
        rospy.Subscriber(
            '/director/scene',
            GenericMessage,
            self.director_scene_mock.record_message
        )
        rospy.Subscriber(
            '/browser_service/right',
            AdhocBrowsers,
            self.common_mock.record_message
        )
        rospy.Subscriber(
            '/browser_service/center',
            AdhocBrowsers,
            self.common_mock.record_message
        )
        rospy.Subscriber(
            '/browser_service/left',
            AdhocBrowsers,
            self.common_mock.record_message
        )
        rospy.Subscriber(
            '/browser_service/browsers',
            AdhocBrowsers,
            self.common_mock.record_message
        )
        rospy.Subscriber(
            '/director/window/ready',
            String,
            self.common_mock.record_message
        )
        rospy.Subscriber(
            '/director/ready',
            Ready,
            self.common_mock.record_message
        )
        rospy.Subscriber(
            '/director/scene',
            GenericMessage,
            self.common_mock.record_message
        )

        rospy.init_node("test_adhoc_browser", anonymous=True)
        rospy.sleep(3)

        # director scene publisher
        self.director_publisher = rospy.Publisher(
            '/director/scene',
            GenericMessage,
            queue_size=3
        )

        try:
            os.mkdir('/tmp/extensions')
        except OSError, e:
            pass

        self.mock_extension_manifest = """
            {
            "manifest_version": 2,

            "name": "Getting started example",
            "description": "This extension shows a Google Image search result for the current page",
            "version": "1.0"
            }
        """
        try:
            os.mkdir('/tmp/extensions/test_extension1')
        except OSError, e:
            pass

        try:
            os.mkdir('/tmp/extensions/test_extension2')
        except OSError, e:
            pass

        try:
            os.mkdir('/tmp/extensions/ros_window_ready')
        except OSError, e:
            pass

        with open('/tmp/extensions/test_extension1/manifest.json', 'w') as ext1_manifest:
            ext1_manifest.write(self.mock_extension_manifest)

        with open('/tmp/extensions/test_extension2/manifest.json', 'w') as ext2_manifest:
            ext2_manifest.write(self.mock_extension_manifest)

    def tearDown(self):
        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)

    def reinitialize_mock_subscribers(self):
        [subscriber.reinitialize() for subscriber in self.subscribers]

    def print_mock_subscribers(self):
        [write_log_to_file("%s's messages: %s" % (subscriber, subscriber.messages)) for subscriber in self.subscribers]

    def test_1_tests_are_working(self):
        """
        """
        self.reinitialize_mock_subscribers()
        self.assertEqual(1, 1)

    def test_2_chrome_extension_initialization(self):
        """
        1. emit browser with extension - check if it got passed to --load-extension arg
        2. emit browser with 2 extensions - check above
        3. emit browser with 2 extensions and prelaoding - ros_window_ready should be loaded implicitly as a first extension
        """

        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_extension_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id,
                         'HSq87uI')
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name, 'test_extension1')
        # 2
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_two_extensions_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id,
                         'ovych17')
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name, 'test_extension1')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[1].name, 'test_extension2')

        # 3
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_two_extensions_and_preloading_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 7)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith('F8qFi0S_'), True)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name,
                         'ros_window_ready',
                         'ros_window_ready didnt get inserted onto exts list as a first extension')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[1].name, 'test_extension1')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[2].name, 'test_extension2')

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.browser_service_mock_center.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages), 2)
        self.assertEqual(len(self.browser_service_mock_right.messages), 2)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_3_chrome_commandline_argument_passing(self):
        """
        1. emit browser with custom command line args - verify that they've been added to cmdargs
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_cmdargs_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id,
                         'HzranO7')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[0].argument,
                         "--disable-out-of-process-pac")
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[1].argument,
                         "--enable-benchmarking")
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[2].argument,
                         "--enable-crash-reporter")
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_4_chrome_user_agent_passing(self):
        """
        1. verify that chrome user agent has been set in commandline args
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_user_agent_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id,
                         'ufFjOQk')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].user_agent,
                         "loltestlmfaorofl")
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_5_chrome_binary_setting(self):
        """
        1. verify that chrome has been attempted to run with a custom binary (make a link)
        """
        try:
            os.symlink("/usr/bin/google-chrome", "/tmp/custom-chrome-binary")
        except Exception:
            pass

        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_binary_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id,
                         '2U4NWMl')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].binary, '/tmp/custom-chrome-binary')

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_6_chrome_persistence(self):
        """
        1. emit one browser without preloading - make service assert
        2. emit the same message again and verify that they havent been updated
        3. emit same message but with a different slug - verify that they havent been touched
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_msg'))
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'r2Dw7jO')

        rospy.wait_for_service('/browser_service/center')
        center_service = rospy.ServiceProxy('/browser_service/center', BrowserPool)
        browsers_on_center = center_service().state

        try:
            browsers_on_center = json.loads(browsers_on_center)
            json_is_valid = True
        except ValueError:
            browsers_on_center = {}
            json_is_valid = False

        self.assertEqual(json_is_valid, True, 'Json returned on /browser_service/center is not valid')
        self.assertEqual(browsers_on_center['r2Dw7jO']['uid'], 'r2Dw7jO')
        browser_timestamp = browsers_on_center['r2Dw7jO']['timestamp']

        # 2
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.common_mock.messages), 10)
        self.assertEqual(len(self.browser_service_mock_center.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages), 2)
        self.assertEqual(len(self.browser_service_mock_right.messages), 2)
        self.assertEqual(len(self.browser_service_mock_common.messages), 2)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'r2Dw7jO')
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id, 'r2Dw7jO')

        rospy.wait_for_service('/browser_service/center')
        center_service = rospy.ServiceProxy('/browser_service/center', BrowserPool)
        browsers_on_center = center_service().state

        try:
            browsers_on_center = json.loads(browsers_on_center)
            json_is_valid = True
        except ValueError:
            browsers_on_center = {}
            json_is_valid = False

        self.assertEqual(json_is_valid, True, 'Json returned on /browser_service/center is not valid')
        self.assertEqual(browsers_on_center['r2Dw7jO']['uid'], 'r2Dw7jO')
        browser_timestamp2 = browsers_on_center['r2Dw7jO']['timestamp']

        self.assertEqual(browser_timestamp, browser_timestamp2, "Emitting same message with identical browser updated the browser instance")

        # 3
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_alt_slug_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.common_mock.messages), 15)
        self.assertEqual(len(self.browser_service_mock_center.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages), 3)
        self.assertEqual(len(self.browser_service_mock_right.messages), 3)
        self.assertEqual(len(self.browser_service_mock_common.messages), 3)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'r2Dw7jO')
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id, 'r2Dw7jO')
        self.assertEqual(self.browser_service_mock_center.messages[2].browsers[0].id, 'r2Dw7jO')

        rospy.wait_for_service('/browser_service/center')
        center_service = rospy.ServiceProxy('/browser_service/center', BrowserPool)
        browsers_on_center = center_service().state

        try:
            browsers_on_center = json.loads(browsers_on_center)
            json_is_valid = True
        except ValueError:
            browsers_on_center = {}
            json_is_valid = False

        self.assertEqual(json_is_valid, True, 'Json returned on /browser_service/center is not valid')
        self.assertEqual(browsers_on_center['r2Dw7jO']['uid'], 'r2Dw7jO')
        browser_timestamp3 = browsers_on_center['r2Dw7jO']['timestamp']

        self.assertEqual(browser_timestamp, browser_timestamp3, "Emitting same message with identical browser but different scene slug updated the browser instance")

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_7_adhoc_browser_preloading(self):
        """
         1.preloading:
          a) emit message with preloading - verify readiness message came, make service assert
          b) emit the same message with preloading again - verify readiness message came, make service assert and confirm that browsers were re-created
          c) emit the same message but with different slug - verify the same as above
          d) emit non-preloaded message with different slug and make usual asserts
         TODO(wz)
          - emit message with preloading - verify readiness message came, make service assert
          - emit non-preloaded message with the same slug and make usual asserts
        """
        # 1a
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 7)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("ZRZAUVz_"), True)
        self.print_mock_subscribers()
        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 1)
        self.assertEqual(self.director_ready_mock.messages[0].instances[0].startswith('ZRZAUVz_'), True)
        self.assertEqual(self.director_window_ready_mock.messages[0].data.startswith('ZRZAUVz_'), True)
        self.assertEqual(len(self.common_mock.messages), 7)
        # get timestmp here

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
        browser_timestamp = browsers_on_center.items()[0][1]['timestamp']

        # 1b
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 15)
        self.assertEqual(len(self.browser_service_mock_center.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages), 2)
        self.assertEqual(len(self.browser_service_mock_right.messages), 2)
        self.assertEqual(len(self.browser_service_mock_common.messages), 2)
        self.assertEqual(len(self.director_window_ready_mock.messages), 2)
        self.assertEqual(len(self.director_ready_mock.messages), 2)
        self.assertEqual(len(self.director_scene_mock.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("ZRZAUVz_"), True)
        self.print_mock_subscribers()
        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 2)
        self.assertEqual(len(self.director_window_ready_mock.messages), 2)
        self.assertEqual(self.director_ready_mock.messages[1].instances[0].startswith('ZRZAUVz_'), True)
        self.assertEqual(self.director_window_ready_mock.messages[1].data.startswith('ZRZAUVz_'), True)

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
        browser_timestamp2 = browsers_on_center.items()[0][1]['timestamp']

        self.assertNotEqual(browser_timestamp, browser_timestamp2)
        self.assertGreater(browser_timestamp2, browser_timestamp)

        # 1c
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_alt_slug_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 21)
        self.assertEqual(len(self.browser_service_mock_center.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages), 3)
        self.assertEqual(len(self.browser_service_mock_right.messages), 3)
        self.assertEqual(len(self.browser_service_mock_common.messages), 3)
        self.assertEqual(len(self.director_window_ready_mock.messages), 3)
        self.assertEqual(len(self.director_ready_mock.messages), 3)
        self.assertEqual(len(self.director_scene_mock.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("ZRZAUVz_"), True)
        self.print_mock_subscribers()
        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 3)
        self.assertEqual(len(self.director_window_ready_mock.messages), 3)
        self.assertEqual(self.director_ready_mock.messages[1].instances[0].startswith('ZRZAUVz_'), True)
        self.assertEqual(self.director_window_ready_mock.messages[1].data.startswith('ZRZAUVz_'), True)

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
        browser_timestamp3 = browsers_on_center.items()[0][1]['timestamp']

        self.assertNotEqual(browser_timestamp2, browser_timestamp3)
        self.assertGreater(browser_timestamp3, browser_timestamp2)

        # 1d
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 26)
        self.assertEqual(len(self.browser_service_mock_center.messages), 4)
        self.assertEqual(len(self.browser_service_mock_left.messages), 4)
        self.assertEqual(len(self.browser_service_mock_right.messages), 4)
        self.assertEqual(len(self.browser_service_mock_common.messages), 4)
        self.assertEqual(len(self.director_window_ready_mock.messages), 3)
        self.assertEqual(len(self.director_ready_mock.messages), 3)
        self.assertEqual(len(self.director_scene_mock.messages), 4)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("ZRZAUVz_"), True)
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id.startswith("ZRZAUVz_"), True)
        self.assertEqual(self.browser_service_mock_center.messages[2].browsers[0].id.startswith("ZRZAUVz_"), True)
        self.assertEqual(self.browser_service_mock_center.messages[3].browsers[0].id, "r2Dw7jO")

        rospy.wait_for_service('/browser_service/center')
        center_service = rospy.ServiceProxy('/browser_service/center', BrowserPool)
        browsers_on_center = center_service().state

        try:
            browsers_on_center = json.loads(browsers_on_center)
            json_is_valid = True
        except ValueError:
            browsers_on_center = {}
            json_is_valid = False

        self.assertEqual(json_is_valid, True, 'Json returned on /browser_service/center is not valid')
        self.assertEqual(browsers_on_center['r2Dw7jO']['uid'], 'r2Dw7jO')
        browser_timestamp = browsers_on_center['r2Dw7jO']['timestamp']

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_8_adhoc_browser_preloading_mix(self):
        """
         1. emit two browsers on one viewport - one with preloading - the other without preloading - verify readiness and service
         2. emit the same message again - non-preloaded browser needs to stay and preloaded browser needs to get re-created
         3. emit the same message again but with different slug - non-preloaded browser should stay, preloaded should get re-created
         TODO(wz):
         4. emit non-preloaded message with different slug and make usual asserts
         5. emit two browsers - one with preloading - the other without preloading - verify readiness and service
         6. emit non-preloaded message with identical slug and make usual asserts
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_two_browsers_with_preloading_mix_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 7)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)

        self.assertEqual(len(self.director_window_ready_mock.messages), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)

        self.assertEqual(len(self.director_scene_mock.messages), 1)

        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_right.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_center.messages[0].browsers), 2)

        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("jJKc4EI_"), True)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[1].id, "CPF_WrJ")

        self.print_mock_subscribers()
        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 2)
        self.assertEqual(len(self.director_ready_mock.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 1)
        self.assertEqual(self.director_ready_mock.messages[0].instances[0].startswith('jJKc4EI_'), True)
        self.assertEqual(self.director_window_ready_mock.messages[0].data.startswith('jJKc4EI_'), True)
        self.assertEqual(len(self.common_mock.messages), 7)

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
        self.assertEqual(len(browsers_on_center), 2)
        if browsers_on_center.items()[0][0].startswith('jJKc4EI_'):
            preloaded_browser_timestamp = browsers_on_center.items()[0][1]['timestamp']
            non_preloaded_browser_timestamp = browsers_on_center.items()[1][1]['timestamp']
        else:
            preloaded_browser_timestamp = browsers_on_center.items()[1][1]['timestamp']
            non_preloaded_browser_timestamp = browsers_on_center.items()[0][1]['timestamp']

        # 2
        self.director_publisher.publish(self.message_factory._get_message('test_two_browsers_with_preloading_mix_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 14)
        self.assertEqual(len(self.browser_service_mock_center.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages), 2)
        self.assertEqual(len(self.browser_service_mock_right.messages), 2)
        self.assertEqual(len(self.browser_service_mock_common.messages), 2)

        self.assertEqual(len(self.director_window_ready_mock.messages), 2)
        self.assertEqual(len(self.director_ready_mock.messages), 2)

        self.assertEqual(len(self.director_scene_mock.messages), 2)

        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_right.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_center.messages[0].browsers), 2)

        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("jJKc4EI_"), True)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[1].id, "CPF_WrJ")

        self.print_mock_subscribers()
        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 2)
        self.assertEqual(len(self.director_ready_mock.messages), 2)
        self.assertEqual(len(self.director_window_ready_mock.messages), 2)
        self.assertEqual(self.director_ready_mock.messages[0].instances[0].startswith('jJKc4EI_'), True)
        self.assertEqual(self.director_window_ready_mock.messages[0].data.startswith('jJKc4EI_'), True)
        self.assertEqual(len(self.common_mock.messages), 14)

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
        self.assertEqual(len(browsers_on_center), 2)

        if browsers_on_center.items()[0][0].startswith('jJKc4EI_'):
            preloaded_browser_timestamp2 = browsers_on_center.items()[0][1]['timestamp']
            non_preloaded_browser_timestamp2 = browsers_on_center.items()[1][1]['timestamp']
        else:
            preloaded_browser_timestamp2 = browsers_on_center.items()[1][1]['timestamp']
            non_preloaded_browser_timestamp2 = browsers_on_center.items()[0][1]['timestamp']

        self.assertNotEqual(preloaded_browser_timestamp2, preloaded_browser_timestamp)
        self.assertEqual(non_preloaded_browser_timestamp2, non_preloaded_browser_timestamp)

        # 3
        self.director_publisher.publish(self.message_factory._get_message('test_two_browsers_with_preloading_mix_alt_slug_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 21)
        self.assertEqual(len(self.browser_service_mock_center.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages), 3)
        self.assertEqual(len(self.browser_service_mock_right.messages), 3)
        self.assertEqual(len(self.browser_service_mock_common.messages), 3)

        self.assertEqual(len(self.director_window_ready_mock.messages), 3)
        self.assertEqual(len(self.director_ready_mock.messages), 3)

        self.assertEqual(len(self.director_scene_mock.messages), 3)

        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_right.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_center.messages[0].browsers), 2)

        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("jJKc4EI_"), True)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[1].id, "CPF_WrJ")

        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 2)
        self.assertEqual(len(self.director_ready_mock.messages), 3)
        self.assertEqual(len(self.director_window_ready_mock.messages), 3)
        self.assertEqual(self.director_ready_mock.messages[0].instances[0].startswith('jJKc4EI_'), True)
        self.assertEqual(self.director_window_ready_mock.messages[0].data.startswith('jJKc4EI_'), True)
        self.assertEqual(len(self.common_mock.messages), 21)

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
        self.assertEqual(len(browsers_on_center), 2)

        if browsers_on_center.items()[0][0].startswith('jJKc4EI_'):
            preloaded_browser_timestamp3 = browsers_on_center.items()[0][1]['timestamp']
            non_preloaded_browser_timestamp3 = browsers_on_center.items()[1][1]['timestamp']
        else:
            preloaded_browser_timestamp3 = browsers_on_center.items()[1][1]['timestamp']
            non_preloaded_browser_timestamp3 = browsers_on_center.items()[0][1]['timestamp']

        self.assertNotEqual(preloaded_browser_timestamp2, preloaded_browser_timestamp3)
        self.assertEqual(non_preloaded_browser_timestamp2, non_preloaded_browser_timestamp3)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.reinitialize_mock_subscribers()

    def test_9_adhoc_browser_custom_preload_event(self):
        """
        Test sending custom event from the extension instead of
        sending readiness message upon standard onDomReady
        """
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_and_custom_preloading_event_msg'))
        rospy.sleep(self.preloading_grace_time)
        self.assertEqual(len(self.common_mock.messages), 7)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)

        self.assertEqual(len(self.director_window_ready_mock.messages), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)

        self.assertEqual(len(self.director_scene_mock.messages), 1)

        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_right.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_center.messages[0].browsers), 1)

        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith("D69tyM"), True)

        self.print_mock_subscribers()
        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 1)
        self.assertEqual(self.director_ready_mock.messages[0].instances[0].startswith('D69tyM'), True)
        self.assertEqual(self.director_window_ready_mock.messages[0].data.startswith('D69tyM'), True)
        self.assertEqual(len(self.common_mock.messages), 7)

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

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowser)
