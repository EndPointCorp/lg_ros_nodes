#!/usr/bin/env python

"""
lg offliner basic offline (not requiring the ROS node itself to run) tests.

"""


import os
import time
import json

import pytest
import rospkg

from std_msgs.msg import Bool
from lg_offliner import ROS_NODE_NAME


class MockTopicPublisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class TestSomethingBasic(object):

    def test_basic(self):
        assert True


if __name__ == "__main__":
    test_pkg = ROS_NODE_NAME
    test_name = "test_lg_offliner_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("%s -s -v --junit-xml=%s" % (test_path, pytest_result_path))
