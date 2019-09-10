#!/usr/bin/env python3

PKG = 'lg_offliner'
NAME = 'test_lg_offliner_when_offline'

import unittest
from multiprocessing import Array

import rospy
from std_msgs.msg import Bool

from lg_offliner import ROS_NODE_NAME
from lg_offliner import LG_OFFLINER_OFFLINE_TOPIC_DEFAULT
from lg_offliner.srv import Offline
from appctl.msg import Mode


RESULT_0 = Array('c', 100)  # size
RESULT_1 = Array('c', 100)  # size
RESULT_2 = Array('c', 100)  # size


class TestLGOfflinerWhenOffline(unittest.TestCase):

    def setUp(self):
        RESULT_0.value = "UNDEFINED"
        RESULT_1.value = "UNDEFINED"
        RESULT_2.value = "UNDEFINED"
        offline_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_OFFLINE_TOPIC_DEFAULT)
        rospy.Subscriber(offline_topic, Bool, self.callback_0)
        # subscribe to topics configured in the .test file
        rospy.Subscriber("/appctl/mode", Mode, self.callback_1)
        rospy.Subscriber("/something/offline", Mode, self.callback_2)

    @staticmethod
    def callback_0(msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        RESULT_0.value = str(msg.data)

    @staticmethod
    def callback_1(msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        RESULT_1.value = msg.mode

    @staticmethod
    def callback_2(msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        RESULT_2.value = msg.mode

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
        assert RESULT_0.value == "True"
        assert RESULT_1.value == "offline"
        assert RESULT_2.value == "offline_scene"
        # can't easily emulate condition becoming online from offline status


if __name__ == "__main__":
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestLGOfflinerWhenOffline)
