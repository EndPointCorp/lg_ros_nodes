#!/usr/bin/env python

"""
lg stats basic offline (not requiring the ROS node itself to run) tests.

Run just this test module manually:
    cd lg_ros_nodes  # must be run from here due to nosetest cfg file
    nosetests --verbosity=3 -s -l DEBUG catkin/src/lg_stats/test/offline/test_lg_stats_basic.py

    nosetestst is troublesome and ignores pytest stuff (e.g. decorators),
        investigate with rosunit <test_module.py>

"""


import os
import time
import json

import pytest
import rospkg

from lg_common.helpers import write_log_to_file
from std_msgs.msg import Bool
from interactivespaces_msgs.msg import GenericMessage
from appctl.msg import Mode
from lg_stats.msg import Session
from lg_stats.msg import Event
from lg_stats import ROS_NODE_NAME
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
                      msg_slot="message.slug",
                      strategy="default")

        out_msg = p._get_outbound_message(in_msg)
        whole_field_name = "message.slug"  # depends on the particular message type above

        # now finally test the method of submitters:
        influx = InfluxDirect.get_data_for_influx(out_msg)
        assert isinstance(influx, dict)
        assert influx["tags"]["topic"] == p.watched_topic
        assert influx["tags"]["field_name"] == whole_field_name
        assert influx["tags"]["value"] == "bbb94866-2216-41a2-83b4-13ba35a3e9dc__scene-3-1"
        influx = InfluxTelegraf.get_data_for_influx(out_msg)
        assert isinstance(influx, str)
        assert p.watched_topic in influx
        assert influx.find("field_name=\"%s\"" % whole_field_name) > -1
        assert influx.find("value=\"%s\"" % out_msg.value) > -1
        # this mock just calls InfluxTelegraf - do the same assertions
        influx = InfluxMock.get_data_for_influx(out_msg)
        assert isinstance(influx, str)
        assert p.watched_topic in influx
        assert influx.find("field_name=\"%s\"" % whole_field_name) > -1
        assert influx.find("value=\"%s\"" % out_msg.value) > -1


class TestLGStatsProcessor(object):
    """
    Test Processor class.
    It's basically offline test, no ROS interaction.

    """
    def test_basic(self):
        p = Processor(watched_topic='/some/topic',
                      resolution=200,
                      msg_slot='test.me')
        assert p.resolution == 200

    def test_publish(self):
        pub = MockTopicPublisher()
        p = Processor(watched_topic="/lol/rofl",
                      msg_slot="application",
                      debug_pub=pub,
                      resolution=200)
        msg1 = Session(application="someapplication1")
        out = p._get_outbound_message(msg1)
        p.debug_pub.publish(out)
        assert isinstance(pub.messages[0], Event)
        assert pub.messages[0].field_name == "application"
        assert pub.messages[0].value == "someapplication1"

    # TODO:
    # need to take slots into account when comparing
    def test_message_comparison(self):
        """
        """
        p = Processor(msg_slot="application",
                      watched_topic="/lol/rofl")
        msg1 = Session(application="someapplication1")
        msg2 = Session(application="someapplication2")
        assert p._compare_messages(msg1, msg1)
        assert not p._compare_messages(msg1, msg2)

    def test_process(self):
        pub = MockTopicPublisher()
        influx = InfluxMock()
        p = Processor(msg_slot="application",
                      watched_topic="/lol/mock",
                      influxdb_client=influx,
                      debug_pub=pub)
        msg1 = Session(application="someapplication1")
        assert p.last_in_msg is None
        p.process(msg1)
        assert p.last_in_msg == msg1

    def test_empty_message_ignored(self):
        """
        Empty message - consider just empty slot of of the slotted incoming message.
        Such shall be ignored.

        """
        # prepare message for testing, as if it was received (incoming message)
        in_msg = GenericMessage(type="json", message=json.loads("{}"))
        p = Processor(watched_topic="/director/scene",
                      msg_slot="message.slug")
        p.process(in_msg)
        # nothing shall happen
        assert p.last_in_msg is None
        assert p.time_of_last_in_msg is None

    def test_get_slot_value(self):
        # prepare message for testing, as if it was received (incoming message)
        in_msg = GenericMessage(type="json", message=json.loads("{}"))
        p = Processor(watched_topic="/director/scene",
                      msg_slot="message.slug")
        # test the same on a lower level
        # empty messages are fine - we just ignore them - dont make them raise exceptions
        # pytest.raises(EmptyIncomingMessage, p._get_slot_value, in_msg)
        subslot = json.loads(real_in_msg_director_scene)["message"]
        in_msg = GenericMessage(type="json", message=json.dumps(subslot))
        assert subslot['slug'] == p._get_slot_value(in_msg)
        in_msg = Session(application="someapplication1")
        p = Processor(watched_topic="loll",
                      msg_slot="application")
        assert p._get_slot_value(in_msg) == "someapplication1"

    def test_background_submission_thread(self):
        msg = GenericMessage(type="json", message=json.loads("""{"slug": "something1234"}"""))
        pub = MockTopicPublisher()
        influx = InfluxMock()
        p = Processor(watched_topic="/director/scene",
                      msg_slot="message.slug",
                      inactivity_resubmission=1,
                      influxdb_client=influx,
                      debug_pub=pub)
        # do the thread testing without ROS intervening
        # without rospy.init_node ... and without threading too ...
        # just invoke the thread worker method
        # nothing has been processed by the processor
        assert p.time_of_last_in_msg is None
        assert p.last_in_msg is None
        # as if the message was received on the ROS topic
        p.process(msg)
        # check that it was correctly processed
        assert len(p.debug_pub.messages) == 1
        assert len(p.influxdb_client.messages) == 1
        assert p.debug_pub.messages[0].field_name == "message.slug"
        assert p.debug_pub.messages[0].value == "something1234"
        assert "/director/scene" in p.influxdb_client.messages[0]
        # how run the thread worker - directly, wait before - there is a time check
        time.sleep(2)
        p._resubmit_worker()
        assert len(p.debug_pub.messages) == 2
        assert len(p.influxdb_client.messages) == 2
        assert p.debug_pub.messages[1].field_name == "message.slug"
        assert p.debug_pub.messages[1].value == "something1234"
        assert "/director/scene" in p.influxdb_client.messages[1]


if __name__ == "__main__":
    # this is ignored by nosetests runner, not by rosunit
    f = open("/tmp/check", 'w')
    f.write(str(time.time()))
    f.close()
    test_pkg = ROS_NODE_NAME
    test_name = "test_lg_stats_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("%s -s -v --junit-xml=%s" % (test_path, pytest_result_path))
