#!/usr/bin/env python

"""
lg_keyboard online tests (the lg_keyboard ROS node is started by rostest ...)

starting roslaunch for development:
    roslaunch --screen lg_keyboard/test/online/test_lg_keyboard.test
    could do:
    py.test -s -v lg_keyboard/test/online/test_lg_keyboard.py

running tests manually:
    rostest lg_keyboard/test/online/test_lg_keyboard.test
        (as long as it contains <test> tag, it's the same as launch file)

"""


import os
import time

import pytest
import rospy
import rospkg

from lg_keyboard import ROS_NODE_NAME


class TestOnlineTest(object):

    def setup_method(self, method):
        pass

    def test_checker(self):
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        assert True


if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    test_pkg = "lg_keyboard"
    test_name = "test_lg_onboard"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
