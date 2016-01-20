#!/usr/bin/env python

"""
lg stats testing scenarios.

starting roslaunch for development:
    roslaunch --screen lg_stats/test/online/test_lg_stats.test

running tests manually:
    rostest lg_stats/test/online/test_lg_stats.test
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

from lg_stats import ROS_NODE_NAME
from lg_stats import get_data


TOPIC_NAME = "/%s/SOMETHING" % ROS_NODE_NAME


class TestLGStats(object):

    @classmethod
    def setup_class(cls):
        pass

    @classmethod
    def teardown_class(cls):
        # TODO
        # do shutdown as seen in rostest.rosrun
        # assert ros services are not run anymore
        # it doesn't seem to be necessary if tests are run by means of the
        # entire package launch file
        # rospy.signal_shutdown('test complete')
        pass

    def setup_method(self, method):
        pass

    def teardown_method(self, _):
        pass

    def test_lg_stats(self):
        """
        Check lg_media topic is there.

        """
        a = get_data()
        print a
        assert a == "taktak"
        assert True
        #master = rosgraph.masterapi.Master("caller")
        #topics = master.getTopicTypes()
        #assert ["/media_service/left_one", "lg_media/AdhocMedias"] in topics
        # topic_type, real_topic, msg_eval = rostopic.get_topic_type(TOPIC_NAME, blocking=False)
        # assert topic_type is not None, "Topic not found: {}".format(TOPIC_NAME)
        # assert topic_type == "%s/AdhocMedias" % ROS_NODE_NAME


if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    # otherwise: FAILURE: test [test_lg_media_basic] did not generate test results

    test_pkg = ROS_NODE_NAME
    test_name = "test_lg_stats"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("%s -s -v --junit-xml=%s" % (test_path, pytest_result_path))
