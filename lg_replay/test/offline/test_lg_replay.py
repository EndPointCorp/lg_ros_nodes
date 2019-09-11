#!/usr/bin/env python3

PKG = 'lg_replay'
NAME = 'test_lg_replay'

import rospy
import unittest
import json

from evdev import InputEvent
from lg_replay import DeviceReplay
from interactivespaces_msgs.msg import GenericMessage


class MockDevice:
    def __init__(self):
        self.events = [
            InputEvent(1441716733, 879280, 3, 0, 9888),
            InputEvent(1441716733, 879280, 3, 1, 15600),
            InputEvent(1441716733, 879280, 0, 0, 0),
            InputEvent(1441716733, 981276, 3, 53, 9872),
            InputEvent(1441716733, 981276, 3, 54, 15664),
            InputEvent(1441716733, 981276, 3, 0, 9872),
            InputEvent(1441716733, 981276, 3, 1, 15664),
            InputEvent(1441716733, 981276, 0, 0, 0),
            InputEvent(1441716733, 982263, 3, 57, -1),
            InputEvent(1441716733, 982263, 1, 330, 0)  # < this event gets tested
        ]

    def read_loop(self):
        return self.events


class MockPublisher:
    def __init__(self):
        self.published_messages = []

    def get_published_messages(self):
        return self.published_messages

    def publish_event(self, message):
        self.published_messages.append(message)


class TestReplay(unittest.TestCase):
    def setUp(self):
        self.mock_device = MockDevice()
        self.mock_publisher = MockPublisher()
        self.replay = DeviceReplay(self.mock_publisher, 'blah', event_ecode='EV_KEY', device=self.mock_device)

    def test_events_get_filtered_and_published(self):
        self.replay.run()
        self.assertEqual(type(self.mock_publisher.get_published_messages()), list)
        self.assertEqual(len(self.mock_publisher.get_published_messages()), 1)
        self.assertEqual(type(self.mock_publisher.get_published_messages()[0]), dict)
        message = self.mock_publisher.get_published_messages()[0]

        self.assertEqual(message['code'], 330)
        self.assertEqual(message['value'], 0)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestReplay)
