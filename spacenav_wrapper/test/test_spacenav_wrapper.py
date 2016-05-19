#!/usr/bin/env python

import rospy
import rosbag
import rostest
import unittest
from geometry_msgs.msg import Twist
from spacenav_wrapper import SpacenavWrapper

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


class TestSpacenavWrapper(unittest.TestCase):
    def setUp(self):
        self.mock = Mock()
        self.wrapper = SpacenavWrapper(twist=self.mock, rezero=self.mock.rezero, full_scale=1)
        self._space_pub = rospy.Publisher(
            '/spacenav/twist', Twist, queue_size=10)
        rospy.Subscriber(
            '/spacenav/twist', Twist, self.wrapper.handle_twist)

    def space_pub(self, msg):
        rospy.sleep(0.1)
        self._space_pub.publish(msg)

    def tearDown(self):
        pass

    def test_1_publish_10_messages(self):
        messages = []
        for i in range(10):
            t = _make_twist(i)
            messages.append(t)
            self.space_pub(t)

        rospy.sleep(1.1)
        self.assertEqual(messages, self.mock.publish_data,
                         "Messages published on on spacenav topic should match "
                         "mock publisher data")

    def test_2_rezero(self):
        gutter_val = 2
        buffer_size = 3
        self.wrapper.gutter_val = gutter_val
        self.wrapper.buffer_size = buffer_size

        self.assertEqual(self.mock.rezero_count, 0,
                         'Rezero count should start at 0')
        for i in range(buffer_size):
            self.space_pub(_make_twist(i))

        self.assertEqual(self.mock.rezero_count, 0,
                         'Rezero count should be at 0 after $buffer_size '
                         'messages that are all different')

        for i in range(buffer_size):
            self.assertEqual(self.mock.rezero_count, 0,
                             'Rezero count should be at 0 after %s messages '
                             'published that are the same' % i)
            self.space_pub(_make_twist(gutter_val + 0.1))
        rospy.sleep(0.1)
        self.assertEqual(self.mock.rezero_count, 1, 'Should have rezeroed')


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
    rostest.rosrun(PKG, NAME, TestSpacenavWrapper)
