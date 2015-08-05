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
import json

import pytest
import rostest
import rospy
import rospkg
import rosgraph.masterapi

from lg_media.srv import MediaAppsInfo
from lg_media.srv import MediaAppsInfoResponse
from lg_media import SRV_QUERY


#class TestMediaService(unittest.TestCase):
class TestMediaService(object):

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
        #rospy.signal_shutdown('test complete')
        pass

    def setup_method(self, method):
        # print "setup_method, running '%s' ..." % method.__name__
        pass

    def teardown_method(self, _):
        pass

    def test_lg_media_service_call(self):
        """
        Assert on presence of service, call the service.

        """
        # problem is - via rostest - requisite ros nodes are run so the test pass,
        #   from within catkin_make run_tests, the ros nodes are (most likely) not
        #   run and the test cases hang

        # pure code tests via nosetets
        # ros interacting tests via .test file and declared via rostest


        #CONTINUE, CONTINUE - FIND THIS:
        # problem: how to harvest correct results from rostests (nosetests are OK)

        # issues in mixing nosetests and rostests (plus rosunit ...)

        rospy.wait_for_service(SRV_QUERY)
        proxy = rospy.ServiceProxy(SRV_QUERY, MediaAppsInfo)
        r = proxy()
        assert isinstance(r, MediaAppsInfoResponse)
        data = json.loads(r.json)
        assert data == {}
        assert data == []

    def test_lg_media_topic_presence(self):
        """
        Check lg_media topic is there.

        """
        #return
        master = rosgraph.masterapi.Master("caller")
        topics = master.getTopicTypes()
        assert ["/media_service/left_one", "lg_media/AdhocMedias"] in topics


if __name__ == "__main__":
    # other lg_ros_nodes use mock, do not interact with ros services via rospy ...

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