#!/usr/bin/env python

"""
lg offliner online scenarios ...

starting roslaunch for development:
    roslaunch --screen lg_offliner/test/online/test_lg_offliner_basic.test
    could do:
    py.test -s -v lg_offliner/test/online/test_lg_offliner_basic.test

running tests manually:
    rostest lg_stats/test/online/test_lg_stats.py
        (as long as it contains <test> tag, it's the same as launch file)


"""


import os
import unittest
import time
import json

import pytest
import rostest
import rospy
import rospkg
import rostopic

from std_msgs.msg import String
from lg_offliner import ROS_NODE_NAME
from lg_offliner import LG_OFFLINER_DEBUG_TOPIC_DEFAULT


class TestLGOfflinerBasic(object):

    def setup_method(self, method):
        pass

    def test_basic(self):
        assert True

if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    test_pkg = ROS_NODE_NAME
    test_name = "test_lg_offliner_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("%s -s -v --junit-xml=%s" % (test_path, pytest_result_path))
