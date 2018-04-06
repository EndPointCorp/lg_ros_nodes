#!/usr/bin/env python

PKG = 'lg_spacenav_globe'
NAME = 'test_lg_spacenav_globe'

import sys
import unittest

import rospy
import rostest

from geometry_msgs.msg import PoseStamped, Twist, Vector3
from lg_spacenav_globe.msg import PortalPose

NAV_ZERO = Twist()
NAV_ONE = Twist(
    linear=Vector3(1, 1, 1),
    angular=Vector3(1, 1, 1),
)


class TestLgSpacenavGlobe(unittest.TestCase):
    def setUp(self):
        self.spacenav_pub = rospy.Publisher(
            '/spacenav/twist',
            Twist,
            queue_size=10,
        )
        self.kiosk_pub = rospy.Publisher(
            '/portal_kiosk/current_pose',
            PoseStamped,
            queue_size=10,
        )

        self.kiosk_pose_msgs = []
        self.kiosk_pose_sub = rospy.Subscriber(
            '/lg_spacenav_globe/kiosk_goto_pose',
            PoseStamped,
            self.kiosk_pose_msgs.append,
        )
        self.display_pose_msgs = []
        self.display_pose_sub = rospy.Subscriber(
            '/lg_spacenav_globe/display_goto_pose',
            PoseStamped,
            self.display_pose_msgs.append,
        )
        self.joystick_msgs = []
        self.joystick_sub = rospy.Subscriber(
            '/joystick/twist',
            Twist,
            self.joystick_msgs.append,
        )

        rospy.sleep(0.5)

    def tearDown(self):
        self.kiosk_pose_sub.unregister()
        self.display_pose_sub.unregister()
        self.joystick_sub.unregister()

    def test_joystick_topic(self):
        """
        Must republish normalized SpaceNav msgs on /joystick/twist.
        """
        self.spacenav_pub.publish(NAV_ZERO)
        rospy.sleep(0.5)
        self.assertEqual(len(self.joystick_msgs), 1)
        self.assertEqual(self.joystick_msgs[0], NAV_ZERO)

        self.spacenav_pub.publish(NAV_ONE)
        rospy.sleep(0.5)
        self.assertEqual(len(self.joystick_msgs), 2)


if __name__ == '__main__':
    rospy.init_node(NAME)

    # startup delay
    rospy.sleep(6.0)

    rostest.rosrun(PKG, NAME, TestLgSpacenavGlobe, sys.argv)
