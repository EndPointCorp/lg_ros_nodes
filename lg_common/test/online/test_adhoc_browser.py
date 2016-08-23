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
        TODO(wz):
         test coverage for adhoc browser pool:
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

         assert types used in below tests:
          - /browser_service/center AdhocBrowsers
          - /browser_service/left AdhocBrowsers
          - /browser_service/right AdhocBrowsers
          - /browser_service/browsers AdhocBrowsers
          - /director/ready Ready
          - /director/window/ready String
          - last but not least: rosservice for adhoc browser pool
        """

        self.message_factory = InteractiveSpacesMessagesFactory()
        self.subscribers = []
        self.browser_service_mock_center = MockSubscriber(topic_name='/browser_service/center')
        self.browser_service_mock_left = MockSubscriber(topic_name='/browser_service/left')
        self.browser_service_mock_right = MockSubscriber(topic_name='/browser_service/right')
        self.browser_service_mock_common = MockSubscriber(topic_name='/browser_service/browsers')
        self.director_window_ready_mock = MockSubscriber(topic_name='/director/window/ready')
        self.director_ready_mock = MockSubscriber(topic_name='/diretory/ready')
        self.director_scene_mock = MockSubscriber(topic_name='/diretory/scene')
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
            self.browser_service_mock_common.record_message
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
        try:
            os.mkdir('/tmp/extensions')
        except OSError, e:
            pass

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

        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_extension'))
        rospy.sleep(1)
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
                         'eO3yVFv')
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name, 'test_extension1')
        # 2
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_two_extensions'))
        rospy.sleep(1)
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
                         'Jm-oj0X')
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name, 'test_extension1')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[1].name, 'test_extension2')
        # 3
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_two_extensions_and_preloading'))
        rospy.sleep(1)
        self.print_mock_subscribers()
        self.assertEqual(len(self.common_mock.messages), 5)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id.startswith('jUSoMIB_'), True)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name,
                         'ros_window_ready',
                         'ros_window_ready didnt get inserted onto exts list as a first extension')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[1].name, 'test_extension1')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[2].name, 'test_extension2')

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers'))
        rospy.sleep(1)
        self.assertEqual(len(self.browser_service_mock_center.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages), 2)
        self.assertEqual(len(self.browser_service_mock_right.messages), 2)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)

    def test_3_chrome_commandline_argument_passing(self):
        """
        1. emit browser with custom command line args - verify that they've been added to cmdargs
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_cmdargs'))
        rospy.sleep(1)
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
                         'cNuJ1A7')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[0].argument,
                         "--disable-out-of-process-pac")
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[1].argument,
                         "--enable-benchmarking")
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[2].argument,
                         "--enable-crash-reporter")
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers'))
        rospy.sleep(1)

    def test_4_chrome_user_agent_passing(self):
        """
        1. verify that chrome user agent has been set in commandline args
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_user_agent'))
        rospy.sleep(1)
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
                         'DVXNLla')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].user_agent,
                         "loltestlmfaorofl")
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers'))
        rospy.sleep(1)

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
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_binary'))
        rospy.sleep(1)
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
                         'GrBcdo_')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].binary, '/tmp/custom-chrome-binary')

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers'))
        rospy.sleep(1)

    def test_6_chrome_persistence(self):
        """
        1. emit one browser without preloading - make service assert
        2. emit the same message again and verify that they havent been updated
        3. emit same message but with a different slug - verify that they havent been touched
        """
        # 1
        self.reinitialize_mock_subscribers()
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center'))
        rospy.sleep(1)
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
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'V0zX4Pj')

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
        self.assertEqual(browsers_on_center['V0zX4Pj']['uid'], 'V0zX4Pj')
        browser_timestamp = browsers_on_center['V0zX4Pj']['timestamp']

        # 2
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center'))
        rospy.sleep(1)
        self.assertEqual(len(self.common_mock.messages), 10)
        self.assertEqual(len(self.browser_service_mock_center.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages), 2)
        self.assertEqual(len(self.browser_service_mock_right.messages), 2)
        self.assertEqual(len(self.browser_service_mock_common.messages), 2)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 2)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'V0zX4Pj')
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id, 'V0zX4Pj')

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
        self.assertEqual(browsers_on_center['V0zX4Pj']['uid'], 'V0zX4Pj')
        browser_timestamp2 = browsers_on_center['V0zX4Pj']['timestamp']

        self.assertEqual(browser_timestamp, browser_timestamp2, "Emitting same message with identical browser updated the browser instance")

        # 3
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_alt_slug'))
        rospy.sleep(1)
        self.assertEqual(len(self.common_mock.messages), 15)
        self.assertEqual(len(self.browser_service_mock_center.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages), 3)
        self.assertEqual(len(self.browser_service_mock_right.messages), 3)
        self.assertEqual(len(self.browser_service_mock_common.messages), 3)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'V0zX4Pj')
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id, 'V0zX4Pj')
        self.assertEqual(self.browser_service_mock_center.messages[2].browsers[0].id, 'V0zX4Pj')

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
        self.assertEqual(browsers_on_center['V0zX4Pj']['uid'], 'V0zX4Pj')
        browser_timestamp3 = browsers_on_center['V0zX4Pj']['timestamp']

        self.assertEqual(browser_timestamp, browser_timestamp3, "Emitting same message with identical browser but different scene slug updated the browser instance")

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers'))
        rospy.sleep(1)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowser)
