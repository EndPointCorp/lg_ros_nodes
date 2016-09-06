#!/usr/bin/env python
PKG = 'lg_common'
NAME = 'test_uscs_service'

import unittest
from lg_common import USCSService
from std_msgs.msg import Bool


class MockPub(object):
    def __init__(self):
        self.published_messages = []

    def publish(self, message):
        self.published_messages.append(message)


class TestManagedApplication(unittest.TestCase):
    def setUp(self):
        self.director_scene_publisher = MockPub()
        self.us = USCSService(
            initial_state_scene_url='',
            on_online_state_scene_url='',
            on_offline_state_scene_url='',
            on_active_state_scene_url='',
            on_inactive_state_scene_url='',
            director_scene_publisher=self.director_scene_publisher
        )

    def test_1_no_messages_published_if_no_urls_specified(self):
        false_message = Bool(data=False)
        true_message = Bool(data=True)
        self.us.handle_connectivity_message(false_message)
        self.assertEqual(len(self.director_scene_publisher.published_messages), 0)
        self.us.handle_connectivity_message(true_message)
        self.assertEqual(len(self.director_scene_publisher.published_messages), 0)
        self.us.handle_activity_message(true_message)
        self.assertEqual(len(self.director_scene_publisher.published_messages), 0)
        self.us.handle_activity_message(false_message)
        self.assertEqual(len(self.director_scene_publisher.published_messages), 0)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestUSCSService)
