#!/usr/bin/env python

PKG = 'lg_mirror'
NAME = 'test_single_touch_sender'

import unittest

import rospy
from evdev import ecodes

from lg_mirror.msg import EvdevEvent, EvdevEvents, ScreenCoordinate

TEST_X = 500
TEST_Y = 600


class Listener:
    def __init__(self):
        self.msgs = []

    def handle_message(self, msg):
        self.msgs.append(msg)


class TestSingleTouchSender(unittest.TestCase):
    def test_single_touch_sender(self):
        listener = Listener()
        rospy.Subscriber('/lg_mirror/fake_touch', ScreenCoordinate,
                         listener.handle_message)

        events_pub = rospy.Publisher('/lg_mirror/touch_events', EvdevEvents,
                                     queue_size=10)

        rospy.sleep(1.0)  # spin-up grace period

        msg = EvdevEvents()

        msg.events.append(EvdevEvent(
            type=ecodes.EV_ABS,
            code=ecodes.ABS_X,
            value=TEST_X
        ))
        msg.events.append(EvdevEvent(
            type=ecodes.EV_ABS,
            code=ecodes.ABS_Y,
            value=TEST_Y
        ))

        events_pub.publish(msg)

        rospy.sleep(1.0)  # pubsub grace period

        self.assertEqual(len(listener.msgs), 1)

        response = listener.msgs[0]
        self.assertEqual(response.x, TEST_X)
        self.assertEqual(response.y, TEST_Y)


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestSingleTouchSender)
