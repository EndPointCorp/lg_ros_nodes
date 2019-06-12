#!/usr/bin/env python

import rospy
import rostest
import unittest
from geometry_msgs.msg import Twist
from spacenav_wrapper import SpacenavRezeroer

PKG = 'spacenav_wrapper'
NAME = 'test_spacenav'


class Mock(object):
    def __init__(self):
        self.publish_data = []
        self.rezero_count = 0

    def publish(self, *args, **kwargs):
        self.publish_data.extend(args)

    def rezero(self, *args, **kwargs):
        self.rezero_count += 1


class TestSpacenavSpinChecker(unittest.TestCase):
    def setUp(self):
        self.mock = Mock()
        self.spin_checker = SpacenavRezeroer(
            threshold=1, seconds_before_rezero=2, rate=1, rezero=self.mock.rezero)
        self._space_pub = rospy.Publisher(
            '/spacenav/twist', Twist, queue_size=10)
        rospy.Subscriber(
            '/spacenav/twist', Twist, self.spin_checker.on_spacenav)

    def space_pub(self, msg):
        self.spin_checker.on_spacenav(msg)

    def test_1_no_rezero(self):
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)
        for i in range(1, 100, self.spin_checker.threshold + 1):
            self.space_pub(_make_twist(i))
            self.spin_checker.on_timer()

        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)

    def test_2_rezero(self):
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)
        # one message should not cause a rezero
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)

        # then the next two spacenav messages should cause a single rezero to happen
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 1,
            "Should have issued a rezero, got %s" % self.mock.rezero_count)

    def test_3_rezero_at_threshold_bound(self):
        # after publishing these two messages, the next message (if within threshold)
        # should cause a rezero
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)
        # making a twist message that will have $THRESHOLD different angular
        # and linear magnitude, this should not rezero
        t = _make_twist(1)
        t.angular.x = 2
        t.linear.x = 2
        self.space_pub(t)
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)

    def test_4_rezero_just_under_threshold_bound(self):
        # after publishing these two messages, the next message (if within threshold)
        # should cause a rezero
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.space_pub(_make_twist(1))
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 0,
            "Should not have issued a rezero, got %s" % self.mock.rezero_count)
        # making a twist message that will have $THRESHOLD different angular
        # and linear magnitude, this should not rezero
        t = _make_twist(1)
        t.angular.x = 2 - 0.000001
        t.linear.x = 2 - 0.000001
        self.space_pub(t)
        self.spin_checker.on_timer()
        self.assertEqual(
            self.mock.rezero_count, 1,
            "Should have issued a rezero, got %s" % self.mock.rezero_count)


def _make_twist(i=0):
    t = Twist()
    t.angular.x = i
    t.angular.y = i
    t.angular.z = i
    t.linear.x = i
    t.linear.y = i
    t.linear.z = i
    return t


if __name__ == '__main__':
    rospy.init_node('test_spacenav_wrap')
    rostest.rosrun(PKG, NAME, TestSpacenavSpinChecker)
