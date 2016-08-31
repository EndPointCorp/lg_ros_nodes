#!/usr/bin/env python

"""
lg offliner online scenarios ...

starting roslaunch for development:
    roslaunch --screen lg_offliner/test/online/test_when_offline.test
    could do:
    py.test -s -v lg_offliner/test/online/test_when_offline.py

running tests manually:
    rostest lg_offliner/test/online/test_when_offline.test
        (as long as it contains <test> tag, it's the same as launch file)

"""


import os
import time
from multiprocessing import Array

import pytest
import rospy
import rospkg
from std_msgs.msg import Bool

from lg_offliner import ROS_NODE_NAME
from lg_offliner import LG_OFFLINER_OFFLINE_TOPIC_DEFAULT
from lg_offliner.srv import Offline
from appctl.msg import Mode


RESULT_0 = Array('c', 100)  # size
RESULT_1 = Array('c', 100)  # size
RESULT_2 = Array('c', 100)  # size


class TestLGOfflinerWhenOffline(object):

    def setup_method(self, method):
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
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        time.sleep(2)
        assert self.get_offline_status() is True
        # corresponding (as configured in the .test file) on offline messages should have
        # been received by now, check:
        assert RESULT_0.value == "True"
        assert RESULT_1.value == "offline"
        assert RESULT_2.value == "offline_scene"
        # can't easily emulate condition becoming online from offline status


if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    test_pkg = ROS_NODE_NAME
    test_name = "test_when_offline"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    #pytest.main("-vv -rfsX -s --junit-xml=%s --cov=. --cov-report term-missing %s" % (pytest_result_path, test_path))
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
