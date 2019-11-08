#!/usr/bin/env python3

"""
lg stats testing scenarios.

starting roslaunch for development:
    roslaunch --screen lg_stats/test/online/test_lg_stats.test
    could do:
    py.test -s -v lg_stats/test/online/test_lg_stats.py

running tests manually:
    rostest lg_stats/test/online/test_lg_stats.test
        (as long as it contains <test> tag, it's the same as launch file)

the time period that the resubmission thread is started:
<param name="inactivity_resubmission" value="60"/>
shall be high so that the spawned threads do not intervene with the
currently executed tests.

"""

PKG = 'lg_stats'
NAME = 'test_lg_stats'

import os
import unittest
import time
import json
import threading
from multiprocessing import Array

import rostest
import rospy
import rospkg
import rostopic

from std_msgs.msg import Bool
from std_msgs.msg import String
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


class TestLGStatsRealMessageChain(unittest.TestCase):
    """
    Test each of the watched topics.
    Configured via roslanch.xml file.
    It's online kind of tests, ROS node is running prior
    to tests: rospy.init_node(ROS_NODE_NAME, anonymous=True) - this
    call initiates connection with ROS master ...

    """

    def setup_method(self, method):
        # subscribed to the /lg_stats/debug topic
        # WARNING: issues retrieving rospy.get_param value from the test file, still
        #   getting the default value (regardless of calling it before or after init_node)
        debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
        self.events = []
        rospy.Subscriber(debug_topic, Event, self.events.append)
        rospy.sleep(1)

    def send_director_scene(self):
        """
        Check stats handling of /director/scene messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = GenericMessage(type="json", message="""{"slug": "something"}""")
        pub = rospy.Publisher("/director/scene", GenericMessage, queue_size=3)
        rospy.sleep(1)
        pub.publish(msg)
        rospy.sleep(1)
        self.assertEqual(self.events[-1].metadata, "something")
        # test with another (real) message
        RESULT.value = b"UNDEFINED"
        slot = json.loads(real_in_msg_director_scene)["message"]
        msg = GenericMessage(type="json", message=json.dumps(slot))
        pub.publish(msg)
        rospy.sleep(1)
        self.assertEqual(self.events[-1].metadata, "bbb94866-2216-41a2-83b4-13ba35a3e9dc__scene-3-1")

    def test_send_activity_active(self):
        """
        Check stats handling of /activity/active messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = Bool(data=True)
        pub = rospy.Publisher("/activity/active", Bool, queue_size=3)
        rospy.sleep(1)
        pub.publish(msg)
        rospy.sleep(1)
        self.assertEqual(self.events[-1].metadata, "True")

    def test_send_activity_inactive(self):
        """
        Check stats handling of /activity/active messages.
        By sending this kind of message, trigger the stats message on /lg_stats/debug.

        """
        msg = Bool(data=False)
        pub = rospy.Publisher("/activity/active", Bool, queue_size=3)
        rospy.sleep(1)
        pub.publish(msg)
        rospy.sleep(1)
        self.assertEqual(self.events[-1].metadata, "False")

    def test_send_panoid(self):
        """
        send panoid and expect metadata to contain it
        """
        msg = String(data='x-12394u8rthjekwqh')
        pub = rospy.Publisher("/streetview/panoid", String, queue_size=3)
        rospy.sleep(1)
        pub.publish(msg)
        rospy.sleep(1)
        self.assertEqual(self.events[-1].metadata, "x-12394u8rthjekwqh")


if __name__ == "__main__":
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestLGStatsRealMessageChain)
