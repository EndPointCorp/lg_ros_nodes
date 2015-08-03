#!/usr/bin/env python

"""
Ad hoc media application manager tests.

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


if __name__ == "__main__":
    # above layer of rostest handles roslaunch file run ros services

    # test class must inherit from unittest.TestCase, not from object
    #rostest.rosrun("lg_media", "test_lg_media_basic", TestMediaService)
    #import sys
    #sys.exit(0)

    # pytest must provide result XML file just as rostest.rosrun would do
    # otherwise: FAILURE: test [test_lg_media_basic] did not generate test results

    # TODO
    # integrate .test roslauch file with package's main roslauch file

    test_pkg = "lg_media"
    test_name = "test_lg_media_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)

    # TODO
    # rename xml_result_path
    #xml_result_path = "/home/xmax/.ros/test_results/lg_media/rosunit-test_lg_media_basic.xml"
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main(['-s', '-v', "--junit-xml=%s" % pytest_result_path])
    #pytest.main(['-s', '-v'])