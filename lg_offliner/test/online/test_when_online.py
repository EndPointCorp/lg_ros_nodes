#!/usr/bin/env python

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


import os
import time

import pytest
import rospy
import rospkg

from lg_offliner import ROS_NODE_NAME
from lg_offliner.srv import Offline


class TestLGOfflinerWhenOnline(object):

    def setup_method(self, method):
        pass

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
        assert self.get_offline_status() is False


if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    test_pkg = ROS_NODE_NAME
    test_name = "test_when_online"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
