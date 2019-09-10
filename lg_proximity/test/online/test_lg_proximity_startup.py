#!/usr/bin/env python3

import unittest
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Bool

PKG = 'lg_proximity'
NAME = 'test_lg_proximity_startup'


class Mock:
    def __init__(self):
        self.state = []

    def record(self, msg):
        self.state.append(msg)


class TestLGProximityStartup(unittest.TestCase):
    def setUp(self):
        self.mock_range = Mock()
        self.mock_presence = Mock()
        rospy.init_node("test_lg_proximity_startup", anonymous=True)
        rospy.Subscriber("/proximity/distance", Range, self.mock_range.record)
        rospy.Subscriber("/proximity/presence", Bool, self.mock_presence.record)
        rospy.sleep(3)

    def test_0_startup_success(self):
        rospy.sleep(1)
        self.assertEqual(self.mock_range.state, [])

    def test_1_startup_success(self):
        rospy.sleep(1)
        self.assertEqual(self.mock_presence.state, [])

    def tearDown(self):
        pass


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestLGProximityStartup)
