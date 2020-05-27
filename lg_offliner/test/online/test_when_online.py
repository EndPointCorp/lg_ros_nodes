#!/usr/bin/env python3

"""
lg offliner online scenarios ...

starting roslaunch for development:
    roslaunch --screen lg_offliner/test/online/test_when_online.test
    could do:
    py.test -s -v lg_offliner/test/online/test_when_online.py

running tests manually:
    rostest lg_offliner/test/online/test_when_online.test
        (as long as it contains <test> tag, it's the same as launch file)

"""

PKG = 'lg_offliner'
NAME = 'test_offliner_when_online'

import unittest

import rospy

from lg_offliner import ROS_NODE_NAME
from lg_msg_defs.srv import Offline


class TestLGOfflinerWhenOnline(unittest.TestCase):

    def setup_method(self, method):
        pass

    def get_offline_status(self):
        """
        Calls the ROS service.

        """
        rospy.wait_for_service("%s/status" % ROS_NODE_NAME)
        proxy = rospy.ServiceProxy("%s/status" % ROS_NODE_NAME, Offline, persistent=False)
        r = proxy()
        res = bool(r.offline)
        return res

    def test_checker(self):
        rospy.sleep(1)
        # first check the initial state - should be False (online) bcoz we assume that
        # the state after relaunch is online - there are no results yet after 1 second
        assert self.get_offline_status() is False
        # now all results should return status code of 0 and we still should be online
        rospy.sleep(10)
        assert self.get_offline_status() is False


if __name__ == "__main__":
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestLGOfflinerWhenOnline)
