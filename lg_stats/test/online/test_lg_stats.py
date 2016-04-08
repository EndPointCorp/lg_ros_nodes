#!/usr/bin/env python

"""
lg stats testing scenarios.

starting roslaunch for development:
    roslaunch --screen lg_stats/test/online/test_lg_stats.test
    could do:
    py.test -s -v lg_stats/test/online/test_lg_stats.py

running tests manually:
    rostest lg_stats/test/online/test_lg_stats.test
        (as long as it contains <test> tag, it's the same as launch file)

"""


import os
import unittest
import time
import json
import threading
from multiprocessing import Array

import pytest
import rostest
import rospy
import rospkg
import rostopic

from lg_common.helpers import write_log_to_file
from std_msgs.msg import Bool
from interactivespaces_msgs.msg import GenericMessage
from appctl.msg import Mode
from lg_stats.msg import Session
from lg_stats.msg import Event
from lg_stats import ROS_NODE_NAME
from lg_stats import LG_STATS_DEBUG_TOPIC_DEFAULT
from lg_stats import Processor
from lg_stats import Submitter
from lg_stats import InfluxDirect
from lg_stats import InfluxTelegraf
from lg_stats import InfluxMock
from lg_stats import EmptyIncomingMessage


RESULT = Array('c', 100)  # size

real_in_msg_director_scene = """
{
"type": "json",
"message": {
  "description": "scene-3-1 desc",
  "duration": 2,
  "name": "scene-3-1",
  "resource_uri": "/director_api/scene/bbb94866-2216-41a2-83b4-13ba35a3e9dc__scene-3-1/",
  "slug": "bbb94866-2216-41a2-83b4-13ba35a3e9dc__scene-3-1",
  "windows": []
  }
}
"""


class TestLGStatsRealMessageChain(object):
    """
    Test each of the watched topics.
    Configured via roslanch.xml file.
    It's online kind of tests, ROS node is running prior
    to tests: rospy.init_node(ROS_NODE_NAME, anonymous=True) - this
    call initiates connection with ROS master ...

    """

    def setup_method(self, method):
        RESULT.value = "UNDEFINED"
        # subscribed to the /lg_stats/debug topic
        # WARNING: issues retrieving rospy.get_param value from the test file, still
        #   getting the default value (regardless of calling it before or after init_node)
        debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
        rospy.Subscriber(debug_topic, Event, self.callback)

    @staticmethod
    def callback(msg):
        """
        Handler of incoming messages on /lg_stats/debug topic.
        Set the shared mem value based on the received message.

        """
        rospy.loginfo("callback received type: '%s'" % type(msg))
        rospy.loginfo(msg)
        RESULT.value = msg.value

    def checker(self, publisher, msg_to_send, expected_value):
        """
        This method is called after init_node
        Wait a bit until the ros infrastructure starts up, sending a message right
        init_node results in a lost message sometimes ...

        """
        rospy.sleep(1)
        publisher.publish(msg_to_send)
        # wait a bit, call back shall set the share mem value accordingly
        for count in range(3):
            if RESULT.value != "UNDEFINED":
                break
            rospy.sleep(1)
        assert RESULT.value == expected_value

    def test_send_director_scene(self):
        """
        Check stats handling of /director/scene messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = GenericMessage(type="json", message="""{"slug": "something"}""")
        pub = rospy.Publisher("/director/scene", GenericMessage, queue_size=3)
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        self.checker(pub, msg, "something")
        # test with another (real) message
        RESULT.value = "UNDEFINED"
        slot = json.loads(real_in_msg_director_scene)["message"]
        msg = GenericMessage(type="json", message=json.dumps(slot))
        self.checker(pub, msg, "bbb94866-2216-41a2-83b4-13ba35a3e9dc__scene-3-1")

    def test_send_appctl_mode(self):
        """
        Check stats handling of /appctl/mode messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = Mode(mode="a good mode")
        pub = rospy.Publisher("/appctl/mode", Mode, queue_size=3)
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        self.checker(pub, msg, "a good mode")

    def test_send_statistics_session(self):
        """
        Check stats handling of /statistics/session messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = Session(application="a good application")
        pub = rospy.Publisher("/statistics/session", Session, queue_size=3)
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        self.checker(pub, msg, "a good application")

    def test_send_activity_active(self):
        """
        Check stats handling of /activity/active messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = Bool(data=True)
        pub = rospy.Publisher("/activity/active", Bool, queue_size=3)
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        self.checker(pub, msg, "True")


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
