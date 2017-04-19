#!/usr/bin/env python
PKG = 'lg_earth'
NAME = 'test_query_writer'

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
        self.volume_controller = VolumeControl(self.mock_pub)

    def test_initial_volume(self):
        """
        Volume is initially clamped between default/2 and default
        """
        self.assertEqual(len(self.mock_pub.data), 1)
        write_log_to_file("%s" % self.mock_pub.data[0])
        self.assertGreaterEqual(self.mock_pub.data[0], self.volume_controller.default / 2)
        self.assertLessEqual(self.mock_pub.data[0], self.volume_controller.default)
