#!/usr/bin/env python

PKG = 'lg_sv'
NAME = 'test_panoid_state'

import rospy
import rostest
import sys
import unittest
from std_msgs.msg import String
from lg_sv.srv import PanoIdState

PUB_PATH = '/streetview/panoid'
SRV_PATH = '/streetview/panoid_state'
TEST_PANOID = 'Taj6kNBbJS5vvm68kANtSw'


class TestPanoIdState(unittest.TestCase):
    def test_state(self):
        panoid_pub = rospy.Publisher(PUB_PATH, String, queue_size=10)

        rospy.wait_for_service(SRV_PATH)
        panoid_srv = rospy.ServiceProxy(SRV_PATH, PanoIdState)

        response = panoid_srv()
        self.assertEqual('', response.panoid)

        panoid_pub.publish(String(TEST_PANOID))

        rospy.sleep(3.0)

        response = panoid_srv()
        self.assertEqual(TEST_PANOID, response.panoid)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestPanoIdState, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
