#!/usr/bin/env python3

PKG = 'lg_offliner'
NAME = 'test_lg_offliner_when_offline'

import unittest

import rospy
from std_msgs.msg import Bool

from lg_offliner import ROS_NODE_NAME
from lg_offliner import LG_OFFLINER_OFFLINE_TOPIC_DEFAULT
from lg_offliner.srv import Offline
from appctl.msg import Mode


class TestLGOfflinerWhenOffline(unittest.TestCase):

    def setUp(self):
        self.results = [None, None, None]
        offline_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_OFFLINE_TOPIC_DEFAULT)
        rospy.Subscriber(offline_topic, Bool, self.callback_0)
        # subscribe to topics configured in the .test file
        rospy.Subscriber("/appctl/mode", Mode, self.callback_1)
        rospy.Subscriber("/something/offline", Mode, self.callback_2)

    def callback_0(self, msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        self.results[0] = msg.data

    def callback_1(self, msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        self.results[1] = msg.mode

    def callback_2(self, msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        self.results[2] = msg.mode

    def get_offline_status(self):
        """
        Calls the ROS service.

        """
        rospy.wait_for_service("%s/status" % ROS_NODE_NAME)
        proxy = rospy.ServiceProxy("%s/status" % ROS_NODE_NAME, Offline)
        r = proxy()
        res = bool(r.offline)
        return res

    def test_checker(self):
        rospy.sleep(4)
        assert self.get_offline_status() is True
        # corresponding (as configured in the .test file) on offline messages should have
        # been received by now, check:
        self.assertTrue(self.results[0])
        self.assertEqual(self.results[1], "offline")
        self.assertEqual(self.results[2], "offline_scene")
        # can't easily emulate condition becoming online from offline status


if __name__ == "__main__":
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestLGOfflinerWhenOffline)
