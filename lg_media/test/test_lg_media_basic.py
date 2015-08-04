#!/usr/bin/env python

"""
Ad hoc media application manager tests.

run tests on an individual package:
    rostest lg_media/launch/dev.launch

run entire lg_ros_notes test suite:
    catkin_make run_tests
    catkin_test_results build/test_results

"""

import os
import unittest

import pytest
import rostest
import rospy
import rospkg


#class TestMediaService(unittest.TestCase):  # object
class TestMediaService(object):

    @classmethod
    def setup_class(cls):
        pass
        # TODO
        # do some asserts on services, ros topics

    @classmethod
    def teardown_class(cls):
        #pass
        # TODO
        # do shutdown as seen in rostest.rosrun
        # assert ros services are not run anymore
        rospy.signal_shutdown('test complete')
        # TODO
        # make some asserts it's all cleared up ...
        # perhaps start tracking ros services proceses per this test class in setup_class
        # check they are all terminated here

    def setup_method(self, method):
        #     print "setup_method, running '%s' ..." % method.__name__
        pass

    def teardown_method(self, _):
        pass

    def test_1(self):
        print "test_1"

    def test_2(self):
        print "test_2"

    def test_3(self):
        print "test_3"
        assert 1 == 2


if __name__ == "__main__":
    # above layer of rostest handles roslaunch file run ros services

    # test class must inherit from unittest.TestCase, not from object
    #rostest.rosrun("lg_media", "test_lg_media_basic", TestMediaService)
    #import sys
    #sys.exit(0)

    # pytest must provide result XML file just as rostest.rosrun would do
    # otherwise: FAILURE: test [test_lg_media_basic] did not generate test results

    test_pkg = "lg_media"
    test_name = "test_lg_media_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)

    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    test_path = os.path.abspath(os.path.dirname(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("%s -s -v --junit-xml=%s" % (test_path, pytest_result_path))
    #pytest.main(['-s', '-v'])