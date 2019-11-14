#!/usr/bin/env python3
import rospy
import unittest
import os
import json

from lg_common.msg import AdhocBrowsers
from lg_common.msg import Ready
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from lg_common import InteractiveSpacesMessagesFactory
from lg_common.helpers import write_log_to_file
from lg_common.srv import BrowserPool
from lg_common.test_helpers import wait_for_assert_equal


PKG = 'lg_common'
NAME = 'test_adhoc_browser'


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
        self.preloading_grace_time = 45
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

        self.subscribers.append(self.browser_service_mock_center)
        self.subscribers.append(self.browser_service_mock_left)
        self.subscribers.append(self.browser_service_mock_right)
        self.subscribers.append(self.browser_service_mock_common)
        self.subscribers.append(self.director_window_ready_mock)
        self.subscribers.append(self.director_ready_mock)
        self.subscribers.append(self.director_scene_mock)

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

        # director scene publisher
        self.director_publisher = rospy.Publisher(
            '/director/scene',
            GenericMessage,
            queue_size=3
        )

        try:
            os.mkdir('/tmp/extensions')
        except OSError:
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
        except OSError:
            pass

        try:
            os.mkdir('/tmp/extensions/test_extension2')
        except OSError:
            pass

        try:
            os.mkdir('/tmp/extensions/ros_window_ready')
        except OSError:
            pass

        with open('/tmp/extensions/test_extension1/manifest.json', 'w') as ext1_manifest:
            ext1_manifest.write(self.mock_extension_manifest)

        with open('/tmp/extensions/test_extension2/manifest.json', 'w') as ext2_manifest:
            ext2_manifest.write(self.mock_extension_manifest)

        self.reinitialize_mock_subscribers()
        rospy.sleep(self.message_emission_grace_time)

    def tearDown(self):
        """
        @ran after each test case
        """
        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time + 3)

    def reinitialize_mock_subscribers(self):
        [subscriber.reinitialize() for subscriber in self.subscribers]

    def print_mock_subscribers(self):
        write_log_to_file("============")
        [write_log_to_file("%s's messages: %s" % (subscriber, subscriber.messages)) for subscriber in self.subscribers]
        write_log_to_file("============")

    def get_browsers_thru_service(self, viewport):
        rospy.wait_for_service('/browser_service/%s' % viewport)
        viewport_service = rospy.ServiceProxy('/browser_service/%s' % viewport, BrowserPool)
        browsers_on_viewport = viewport_service('{}').state

        try:
            browsers_on_viewport = json.loads(browsers_on_viewport)
            json_is_valid = True
        except ValueError:
            browsers_on_viewport = {}
            json_is_valid = False

        self.assertEqual(json_is_valid, True, 'Json returned on /browser_service/%s is not valid' % viewport)

        return browsers_on_viewport

    def test1_tests_are_working(self):
        """
        dummy test to see if stuff initializes properly
        """
        self.assertEqual(1, 1)

    def test2_chrome_one_extension_initialization(self):
        """
        emit browser with extension - check if it got passed to --load-extension arg
        """
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_extension_msg'))
        rospy.sleep(self.message_emission_grace_time)

        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name, 'test_extension1')
        browsers_on_center = self.get_browsers_thru_service('center')
        # Two service extensions + extensions from message
        self.assertEqual('test_extension1' in ' '.join(list(browsers_on_center.items())[0][1]['extensions']), True)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)

    def test2a_chrome_extension_initialization_with_two_extensions(self):
        """
        emit browser with 2 extensions - test_extension and ros_window_ready
        """
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_two_extensions_and_preloading_msg'))
        wait_for_assert_equal(len(self.director_window_ready_mock.messages), 1, timeout=self.preloading_grace_time)

        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[0].name, 'test_extension1')
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].extensions[1].name, 'test_extension2')

        browsers_on_center = self.get_browsers_thru_service('center')
        # Two extensions, +ros_window_ready +url_monitor
        self.assertEqual(len(list(browsers_on_center.items())[0][1]['extensions']), 4)

        # cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(self.message_emission_grace_time)

    def test3_chrome_commandline_argument_passing(self):
        """
        1. emit browser with custom command line args - verify that they've been added to cmdargs
        """
        # 1
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_cmdargs_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[0].argument,
                         "--disable-out-of-process-pac")
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[1].argument,
                         "--enable-benchmarking")
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].command_line_args[2].argument,
                         "--enable-crash-reporter")
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)

        wait_for_assert_equal(len(list(self.get_browsers_thru_service('center').items())), 1, self.preloading_grace_time)

        browsers_on_center = self.get_browsers_thru_service('center')

        self.assertEqual('--disable-out-of-process-pac' in list(browsers_on_center.items())[0][1]['command_line_args'], True)
        self.assertEqual('--enable-benchmarking' in list(browsers_on_center.items())[0][1]['command_line_args'], True)
        self.assertEqual('--enable-crash-reporter' in list(browsers_on_center.items())[0][1]['command_line_args'], True)

    def test4_chrome_user_agent_passing(self):
        """
        1. verify that chrome user agent has been set in commandline args
        """
        # 1
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_user_agent_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.browser_service_mock_center.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages), 1)
        self.assertEqual(len(self.browser_service_mock_right.messages), 1)
        self.assertEqual(len(self.browser_service_mock_common.messages), 1)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].user_agent,
                         "loltestlmfaorofl")
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)

        browsers_on_center = self.get_browsers_thru_service('center')

        self.assertEqual(list(browsers_on_center.items())[0][1]['user_agent'], 'loltestlmfaorofl')

    def test5_chrome_binary_setting(self):
        """
        1. verify that chrome has been attempted to run with a custom binary e.g. beta
        """
        # 1
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_custom_binary_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].version, 'beta')

        browsers_on_center = self.get_browsers_thru_service('center')

        self.assertEqual(list(browsers_on_center.items())[0][1]['binary'], '/usr/bin/google-chrome-beta')

    def test6_chrome_persistence(self):
        """
        1. emit one browser without preloading - make service assert
        2. emit the same message again and verify that they havent been updated
        3. emit same message but with a different slug - verify that they havent been touched
        """

        # 1
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_msg'))
        rospy.sleep(self.message_emission_grace_time)
        wait_for_assert_equal(len(self.director_scene_mock.messages), 1, self.preloading_grace_time)

        # no director window ready should be published
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'gzJERcJ')

        browsers_on_center = self.get_browsers_thru_service('center')
        # self.assertEqual(browsers_on_center, 'asd')
        browser_timestamp_before = browsers_on_center['gzJERcJ']['timestamp']
        self.assertEqual(browsers_on_center['gzJERcJ']['uid'], 'gzJERcJ')

        # 2
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_msg'))
        rospy.sleep(self.message_emission_grace_time)
        wait_for_assert_equal(len(self.director_scene_mock.messages), 2, self.preloading_grace_time)
        self.assertEqual(len(self.director_window_ready_mock.messages), 0)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'gzJERcJ')
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id, 'gzJERcJ')

        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(list(browsers_on_center.items())), 1)

        browser_timestamp_after = list(browsers_on_center.items())[0][1]['timestamp']
        self.assertEqual(browsers_on_center['gzJERcJ']['uid'], 'gzJERcJ')

        self.assertEqual(browser_timestamp_before, browser_timestamp_after, "Emitting same message with identical browser updated the browser instance")

        # 3
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_alt_slug_msg'))
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(len(self.director_ready_mock.messages), 0)
        self.assertEqual(len(self.director_scene_mock.messages), 3)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(self.browser_service_mock_center.messages[0].browsers[0].id, 'gzJERcJ')
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id, 'gzJERcJ')
        self.assertEqual(self.browser_service_mock_center.messages[2].browsers[0].id, 'gzJERcJ')

        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(list(browsers_on_center.items())), 1)

        self.assertEqual(browsers_on_center['gzJERcJ']['uid'], 'gzJERcJ')
        self.assertEqual(browser_timestamp_before, browser_timestamp_after, "Emitting same message with identical browser updated the browser instance")

        browser_timestamp_even_after = list(browsers_on_center.items())[0][1]['timestamp']
        self.assertEqual(browser_timestamp_before, browser_timestamp_even_after, "Emitting same message with identical browser but different scene slug updated the browser instance")

    def test7_adhoc_browser_preloading(self):
        """
         1.preloading:
          a) emit message with preloading - verify readiness message came, make service assert
          b) emit the same message with preloading again - verify readiness message came, make service assert and confirm that browsers were re-created
          c) emit the same message but with different slug - verify the same as above
          d) emit non-preloaded message with different slug and make usual asserts
        """
        # 1a
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_msg'))
        rospy.sleep(self.message_emission_grace_time)
        wait_for_assert_equal(len(self.director_window_ready_mock.messages) > 0, True, self.preloading_grace_time)
        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)

        browsers_on_center = self.get_browsers_thru_service('center')
        browser_timestamp = list(browsers_on_center.items())[0][1]['timestamp']

        # 1b
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_msg'))
        rospy.sleep(self.message_emission_grace_time)
        wait_for_assert_equal(len(self.director_ready_mock.messages), 2, self.preloading_grace_time)

        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(browsers_on_center), 1)
        browser_timestamp2 = list(browsers_on_center.items())[0][1]['timestamp']

        self.assertNotEqual(browser_timestamp, browser_timestamp2)
        self.assertGreater(browser_timestamp2, browser_timestamp)

        # 1c
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_alt_slug_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 3, self.preloading_grace_time)
        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(browsers_on_center), 1)
        browser_timestamp3 = list(browsers_on_center.items())[0][1]['timestamp']

        self.assertNotEqual(browser_timestamp2, browser_timestamp3)
        self.assertGreater(browser_timestamp3, browser_timestamp2)

        # 1d
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_on_center_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 3, self.preloading_grace_time)
        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(browsers_on_center), 1)

        browser_timestamp4 = list(browsers_on_center.items())[0][1]['timestamp']

        self.assertNotEqual(browser_timestamp3, browser_timestamp4)
        self.assertGreater(browser_timestamp4, browser_timestamp3)

    def test8_adhoc_browser_preloading_mix(self):
        """
         1. emit two browsers on one viewport - one with preloading - the other without preloading - verify readiness and service
         2. emit the same message again - non-preloaded browser needs to stay and preloaded browser needs to get re-created
         3. emit the same message again but with different slug - non-preloaded browser should stay, preloaded should get re-created
         TODO(wz):
         4. emit non-preloaded message with different slug and make usual asserts
         5. emit two browsers - one with preloading - the other without preloading - verify readiness and service
         6. emit non-preloaded message with identical slug and make usual asserts

         - P - preloaded
         - NP - preloaded

            - P NP
            - P NP
            - P + NP (alt slug)
            - P + NP (alt slug)
            - NP + NP
            - NP + NP
            - NP + NP (alt slug)
            - NP + NP (alg slug)
        """
        # 1
        self.director_publisher.publish(self.message_factory._get_message('test_two_browsers_with_preloading_mix_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 1, self.preloading_grace_time + 15)

        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(browsers_on_center), 2)

        extensions_on_first_browser = list(browsers_on_center.items())[0][1].get('extensions', None)
        extensions_on_second_browser = list(browsers_on_center.items())[1][1].get('extensions', None)

        if extensions_on_first_browser and 'ros_window_ready' in ' '.join(list(browsers_on_center.items())[0][1]['extensions']):
            preloaded_browser_timestamp = list(browsers_on_center.items())[0][1]['timestamp']
            non_preloaded_browser_timestamp = list(browsers_on_center.items())[1][1]['timestamp']
        elif extensions_on_second_browser and 'ros_window_ready' in ' '.join(list(browsers_on_center.items())[1][1]['extensions']):
            preloaded_browser_timestamp = list(browsers_on_center.items())[1][1]['timestamp']
            non_preloaded_browser_timestamp = list(browsers_on_center.items())[0][1]['timestamp']

        # 2
        self.director_publisher.publish(self.message_factory._get_message('test_two_browsers_with_preloading_mix_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 2, self.preloading_grace_time + 15)

        browsers_on_center = self.get_browsers_thru_service('center')
        self.assertEqual(len(browsers_on_center), 2)

        extensions_on_first_browser = list(browsers_on_center.items())[0][1].get('extensions', None)
        extensions_on_second_browser = list(browsers_on_center.items())[1][1].get('extensions', None)

        if extensions_on_first_browser and 'ros_window_ready' in ' '.join(list(browsers_on_center.items())[0][1]['extensions']):
            preloaded_browser_timestamp2 = list(browsers_on_center.items())[0][1]['timestamp']
            non_preloaded_browser_timestamp2 = list(browsers_on_center.items())[1][1]['timestamp']
        elif extensions_on_second_browser and 'ros_window_ready' in ' '.join(list(browsers_on_center.items())[1][1]['extensions']):
            preloaded_browser_timestamp2 = list(browsers_on_center.items())[1][1]['timestamp']
            non_preloaded_browser_timestamp2 = list(browsers_on_center.items())[0][1]['timestamp']

        self.assertNotEqual(preloaded_browser_timestamp2, preloaded_browser_timestamp)
        self.assertEqual(non_preloaded_browser_timestamp2, non_preloaded_browser_timestamp)

        # 3
        self.director_publisher.publish(self.message_factory._get_message('test_two_browsers_with_preloading_mix_alt_slug_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 3, self.preloading_grace_time + 15)

        browsers_on_center = self.get_browsers_thru_service('center')

        extensions_on_first_browser = list(browsers_on_center.items())[0][1].get('extensions', None)
        extensions_on_second_browser = list(browsers_on_center.items())[1][1].get('extensions', None)

        if extensions_on_first_browser and 'ros_window_ready' in ' '.join(list(browsers_on_center.items())[0][1]['extensions']):
            preloaded_browser_timestamp3 = list(browsers_on_center.items())[0][1]['timestamp']
            non_preloaded_browser_timestamp3 = list(browsers_on_center.items())[1][1]['timestamp']
        elif extensions_on_second_browser and 'ros_window_ready' in ' '.join(list(browsers_on_center.items())[1][1]['extensions']):
            preloaded_browser_timestamp3 = list(browsers_on_center.items())[1][1]['timestamp']
            non_preloaded_browser_timestamp3 = list(browsers_on_center.items())[0][1]['timestamp']

        self.assertNotEqual(preloaded_browser_timestamp2, preloaded_browser_timestamp3)
        self.assertEqual(non_preloaded_browser_timestamp2, non_preloaded_browser_timestamp3)

    def test9_adhoc_browser_custom_preload_event(self):
        """
        Test sending custom event from the extension instead of
        sending readiness message upon standard onDomReady
        """

        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_and_custom_preloading_event_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 1, self.preloading_grace_time)

        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 1)
        self.assertEqual(len(self.director_ready_mock.messages), 1)

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

    def test9b_adhoc_browser_readiness_handbrake(self):
        """
        Tests one browser with wrong URL

        Ready message should come after readiness handbrake timeout of 10 seconds (default)
        """
        self.director_publisher.publish(self.message_factory._get_message('test_one_browser_with_preloading_and_wrong_url_msg'))
        rospy.sleep(self.message_emission_grace_time)

        wait_for_assert_equal(len(self.director_ready_mock.messages), 1, self.preloading_grace_time)

        self.assertEqual(len(self.director_scene_mock.messages), 1)
        self.assertEqual(len(self.browser_service_mock_left.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_right.messages[0].browsers), 0)
        self.assertEqual(len(self.browser_service_mock_center.messages[0].browsers), 1)

        self.assertEqual(len(self.browser_service_mock_common.messages[0].browsers), 1)

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

    def test9a_browser_id_is_predictable(self):
        """
        emit browser message twice. ID of two browsers should be identical
        """
        self.director_publisher.publish(
            self.message_factory._get_message(
                'test_one_browser_with_two_extensions_and_preloading_msg'
            )
        )
        rospy.sleep(self.message_emission_grace_time)

        browser_prefix = self.browser_service_mock_center.messages[0].browsers[0].id.split('_')[0]

        self.director_publisher.publish(
            self.message_factory._get_message(
                'test_one_browser_with_two_extensions_and_preloading_msg'
            )
        )
        rospy.sleep(self.message_emission_grace_time)
        self.assertEqual(self.browser_service_mock_center.messages[1].browsers[0].id.startswith(browser_prefix), True)


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_adhoc_browser", anonymous=True)
    rostest.rosrun(PKG, NAME, TestAdhocBrowser)
