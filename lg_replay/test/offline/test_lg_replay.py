#!/usr/bin/env python

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
                InputEvent(1441716733L, 879280L, 3, 0, 9888L),
                InputEvent(1441716733L, 879280L, 3, 1, 15600L),
                InputEvent(1441716733L, 879280L, 0, 0, 0L),
                InputEvent(1441716733L, 981276L, 3, 53, 9872L),
                InputEvent(1441716733L, 981276L, 3, 54, 15664L),
                InputEvent(1441716733L, 981276L, 3, 0, 9872L),
                InputEvent(1441716733L, 981276L, 3, 1, 15664L),
                InputEvent(1441716733L, 981276L, 0, 0, 0L),
                InputEvent(1441716733L, 982263L, 3, 57, -1L),
                InputEvent(1441716733L, 982263L, 1, 330, 0L) # < this event gets tested
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

        self.assertEqual(message['scancode'], 330)
        self.assertEqual(message['keystate'], 0)
        self.assertEqual(message['keycode'], 'BTN_TOUCH')

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestReplay)
