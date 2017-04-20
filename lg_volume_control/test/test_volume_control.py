#!/usr/bin/env python
PKG = 'lg_volume_control'
NAME = 'test_volume_control'

import unittest
import os

from lg_volume_control import VolumeControl
from std_msgs.msg import UInt8, Int8

from lg_common.helpers import write_log_to_file


class MockPub:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg)


class TestVolumeControl(unittest.TestCase):
    def setUp(self):
        self.mock_pub = MockPub()
        # setting scale to 1 to make tests easier to read
        self.volume_controller = VolumeControl(self.mock_pub, scale=1)

    def test_initial_volume(self):
        """
        Volume is initially clamped between default/2 and default
        """
        self.assertEqual(len(self.mock_pub.data), 1)
        write_log_to_file("%s" % self.mock_pub.data[0])
        self.assertGreaterEqual(self.mock_pub.data[0], self.volume_controller.default / 2)
        self.assertLessEqual(self.mock_pub.data[0], self.volume_controller.default)

    def test_max_volume(self):
        """
        Grab the initial data from self.data[0] and then add just enough to be 100
        and then check that the new volume is 100, then add more than 100 and
        check that the volume returned does not exceed 100
        """
        current_volume = self.mock_pub.data[0]
        # calculate the ammount needed to increment the current volume
        increment = 100 - current_volume

        # craft a message and call the handle_volume callback
        increment_message = Int8()
        increment_message.data = increment
        self.volume_controller.handle_volume_change_request(increment_message)

        # check that we are at 100 volume now
        self.assertEqual(self.mock_pub.data[-1], 100)

        # increment another 1 volume
        increment_message.data = 1
        self.volume_controller.handle_volume_change_request(increment_message)

        # check that we are _still_ 100 (we should never go above 100 volume)
        self.assertEqual(self.mock_pub.data[-1], 100)

    def test_min_volume(self):
        """
        similar to test_max_volume, but checking that we don't go below 0
        """
        current_volume = self.mock_pub.data[0]
        # calculate the ammount needed to increment the current volume
        increment = 0 - current_volume

        # craft a message and call the handle_volume callback
        increment_message = Int8()
        increment_message.data = increment
        self.volume_controller.handle_volume_change_request(increment_message)

        # check that we are at 0  volume now
        self.assertEqual(self.mock_pub.data[-1], 0)

        # decrement another 1 volume
        increment_message.data = -1
        self.volume_controller.handle_volume_change_request(increment_message)

        # check that we are _still_ 0 (we should never go below 0 volume)
        self.assertEqual(self.mock_pub.data[-1], 0)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestQueryWriter)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
