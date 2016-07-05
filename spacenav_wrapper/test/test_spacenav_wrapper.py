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
        self.wrapper = SpacenavWrapper(twist=self.mock)
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
