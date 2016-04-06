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

TODO:
    -test empty message (watched field empty of relevant slot empty)
    -need to test all branches involved in slot return

"""


import os
import unittest
import time
import json
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
from lg_stats.submitters import Submitter
from lg_stats.submitters import InfluxDirect
from lg_stats.submitters import InfluxTelegraf
from lg_stats.submitters import InfluxMock


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


class MockTopicPublisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class TestSubmitters(object):

    def test_submitters_instantiation(self):
        pytest.raises(RuntimeError, Submitter)
        InfluxDirect()
        InfluxTelegraf()
        InfluxMock()

    def test_get_data_for_influx(self):
        # prepare message for testing, as if it was received (incoming message)
        slot = json.loads(real_in_msg_director_scene)["message"]
        in_msg = GenericMessage(type="json", message=json.dumps(slot))
        # get outgoing message (to /lg_stats/debug) - via Processor instance
        p = Processor(watched_topic="/director/scene",
                      msg_slot="message",
                      watched_field_name="slug")
        out_msg = p.get_outbound_message(in_msg)
        whole_field_name = "message.slug"  # depends on the particular message type above
        # now finally test the method of submitters:
        influx = InfluxDirect.get_data_for_influx(out_msg)
        assert isinstance(influx, dict)
        assert influx["measurement"] == p.watched_topic
        assert influx["tags"]["field_name"] == whole_field_name
        assert influx["tags"]["value"] == out_msg.value
        influx = InfluxTelegraf.get_data_for_influx(out_msg)
        assert isinstance(influx, str)
        assert influx.startswith(p.watched_topic)
        assert influx.find("field_name=%s" % whole_field_name) > -1
        assert influx.find("value=%s" % out_msg.value) > -1
        # this mock just calls InfluxTelegraf - do the same assertions
        influx = InfluxMock.get_data_for_influx(out_msg)
        assert isinstance(influx, str)
        assert influx.startswith(p.watched_topic)
        assert influx.find("field_name=%s" % whole_field_name) > -1
        assert influx.find("value=%s" % out_msg.value) > -1


class TestLGStatsProcessor(object):
    """
    Test Processor class.

    """
    def test_basic(self):
        p = Processor(resolution=200)
        assert p.resolution == 200

    def test_publish(self):
        pub = MockTopicPublisher()
        p = Processor(watched_topic=None,
                      watched_field_name="application",
                      debug_pub=pub,
                      resolution=200)
        msg1 = Session(application="someapplication1")
        out = p.get_outbound_message(msg1)
        p.debug_pub.publish(out)
        assert isinstance(pub.messages[0], Event)
        assert pub.messages[0].field_name == "application"
        assert pub.messages[0].value == "someapplication1"

    def test_message_comparison(self):
        """
        TODO:
            need to take slots into account when comparing
        """
        p = Processor(watched_field_name="application")
        msg1 = Session(application="someapplication1")
        msg2 = Session(application="someapplication2")
        assert p.compare_messages(msg1, msg1)
        assert not p.compare_messages(msg1, msg2)

    def test_process(self):
        rospy.init_node(ROS_NODE_NAME, anonymous=True)  # needed by ROS time
        pub = MockTopicPublisher()
        influx = InfluxMock()
        p = Processor(watched_field_name="application",
                      influxdb_client=influx,
                      debug_pub=pub)
        msg1 = Session(application="someapplication1")
        assert p.last_in_msg is None
        p.process(msg1)
        assert p.last_in_msg == msg1

    @pytest.mark.skipif(True, reason="Resolution behaviour will be changing (#126)")
    def test_process_with_resolution_check(self):
        """
        This tests whole Processor chain.

        """
        rospy.init_node(ROS_NODE_NAME, anonymous=True)  # needed by ROS time
        pub = MockTopicPublisher()
        influx = InfluxMock()
        p = Processor(watched_field_name="application",
                      debug_pub=pub,
                      resolution=1,
                      influxdb_client=influx)
        msg1 = Session(application="someapplication1")
        assert p.last_msg is None
        p.process(msg1)
        assert p.last_msg == msg1
        msg2 = Session(application="someapplication1")
        p.process(msg2)
        assert p.last_msg == msg2
        assert pub.messages == []
        rospy.sleep(1.1)
        msg3 = Session(application="someapplication1")
        p.process(msg2)
        assert p.last_msg == msg3
        assert pub.messages[0].value == "someapplication1"


class TestLGStatsRealMessageChain(object):
    """
    Test each of the watched topics.
    Configured via roslanch.xml file.
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

    @pytest.mark.skipif(True, reason="Intervening with previous /statistics/session test. Will be chiefly "
                        "reimplemented anyway, since the resolution behaviour will be changing (#126)")
    def test_resolution_period(self):
        """
        All above tests send a message twice and the second one is sent
        after the resolution period (configured value). The first message is
        only processed if the relevant field is the same in the second message
        and AFTER resolution time.
        Test that sending second message EARLIER than the resolution
        period will result in NO stats reaction.
        """
        msg = Session(application="a good application")
        pub = rospy.Publisher("/statistics/session", Session, queue_size=3)
        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        RESULT.value = "won't change"
        rospy.sleep(2)
        pub.publish(msg)
        # wait shorter time than resolution period:
        #   <param name="resolution" value="1"/>
        rospy.sleep(0.1)
        pub.publish(msg)
        # give ROS some time to process
        rospy.sleep(2)
        assert RESULT.value == "won't change"


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
