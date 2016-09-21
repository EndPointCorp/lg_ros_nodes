#!/usr/bin/env python

PKG = 'lg_common'
NAME = 'test_uscs_service'

import time
import rospy
import unittest
import requests
import json

from lg_common import USCSService
from std_msgs.msg import Bool
from lg_common.srv import InitialUSCS
from interactivespaces_msgs.msg import GenericMessage


class MockSub:
    def __init__(self, topic):
        self.topic = topic
        self.state = []

    def handle_message(self, msg):
        self.state.append(msg)


class TestUSCSService(unittest.TestCase):
    """
    Test cases:
    - startup with scene urls
     - make an assert that service got initial state
     - publish active, inactive, offline and online messages
     - make asserts that after each message, director scene was published
    """
    def setUp(self):
        self.initial_state_url = 'http://127.0.0.1:8008/lg_common/webapps/uscs_messages/initial_state.json'
        self.on_online_state_url = 'http://127.0.0.1:8008/lg_common/webapps/uscs_messages/on_online.json'
        self.on_offline_state_url = 'http://127.0.0.1:8008/lg_common/webapps/uscs_messages/on_offline.json'
        self.on_active_state_url = 'http://127.0.0.1:8008/lg_common/webapps/uscs_messages/on_active.json'
        self.on_inactive_state_url = 'http://127.0.0.1:8008/lg_common/webapps/uscs_messages/on_inactive.json'

        self.director_mock_subscriber = MockSub(topic='/director/scene')
        self.activity_mock_subscriber = MockSub(topic='/activity/active')
        self.connectivity_mock_subscriber = MockSub(topic='/lg_offliner_offline')

        self.activity_publisher = rospy.Publisher('/activity/active', Bool, queue_size=3)
        self.connectivity_publisher = rospy.Publisher('/lg_offliner/offline', Bool, queue_size=3)

        rospy.Subscriber(
            '/director/scene',
            GenericMessage,
            self.director_mock_subscriber.handle_message
        )

        rospy.Subscriber(
            '/activity/active',
            Bool,
            self.activity_mock_subscriber.handle_message
        )

        rospy.Subscriber(
            '/lg_offliner/offline',
            Bool,
            self.connectivity_mock_subscriber.handle_message
        )

        rospy.init_node("test_uscs_service_mocks", anonymous=True)
        self.msg_emission_grace_time = 1
        rospy.sleep(3)

    def tearDown(self):
        pass

    def test_0_setup(self):
        self.assertEqual(len(self.director_mock_subscriber.state), 0)
        self.assertEqual(len(self.connectivity_mock_subscriber.state), 0)
        self.assertEqual(len(self.activity_mock_subscriber.state), 0)

    def test_1_initial_state(self):
        desired_initial_state = json.loads(requests.get(self.initial_state_url).content)

        rospy.wait_for_service('/initial_state')
        initial_state_service = rospy.ServiceProxy('/initial_state', InitialUSCS)
        initial_state = initial_state_service().message
        initial_state = json.loads(initial_state)
        self.assertEqual(initial_state, desired_initial_state)

    def test_2_online_state(self):
        """
        """
        self.connectivity_publisher.publish(Bool(data=True))
        rospy.sleep(self.msg_emission_grace_time)
        self.assertEqual(len(self.connectivity_mock_subscriber.state), 1)
        self.assertEqual(len(self.activity_mock_subscriber.state), 0)
        self.assertEqual(len(self.director_mock_subscriber.state), 1)

    def test_3_offline_state(self):
        """
        """
        self.connectivity_publisher.publish(Bool(data=False))
        rospy.sleep(self.msg_emission_grace_time)
        self.assertEqual(len(self.connectivity_mock_subscriber.state), 1)
        self.assertEqual(len(self.activity_mock_subscriber.state), 0)
        self.assertEqual(len(self.director_mock_subscriber.state), 1)

    def test_4_active_state(self):
        """
        Sending `active=True` should not send a message because
        uscs service is active by default after initialization
        """
        self.activity_publisher.publish(Bool(data=True))
        rospy.sleep(self.msg_emission_grace_time)
        self.assertEqual(len(self.connectivity_mock_subscriber.state), 0)
        self.assertEqual(len(self.activity_mock_subscriber.state), 1)
        self.assertEqual(len(self.director_mock_subscriber.state), 0)

    def test_5_inactive_state(self):
        """
        """
        self.activity_publisher.publish(Bool(data=False))
        rospy.sleep(self.msg_emission_grace_time)
        self.assertEqual(len(self.connectivity_mock_subscriber.state), 0)
        self.assertEqual(len(self.activity_mock_subscriber.state), 1)
        self.assertEqual(len(self.director_mock_subscriber.state), 1)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestUSCSService)
