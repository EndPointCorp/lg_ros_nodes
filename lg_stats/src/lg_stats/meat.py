#!/usr/bin/env python
"""
NOTES:

- Session.msg
    borrowed from statistics/msg/Session.msg
    it's possible to introduce this dependency (or leave it as is)

- check statistics/StatsD.msg

Will need to define association between source topics and type of
    result statistical data (whether it's event based or time-series based).
    This will need to be either configured (not sure if "output-type" will
    be easy to blend with activity_sources processing. Or hard-coded in an
    internal relational matrix (output msg fields: type, application).

TODO:
    - upon node shutdown - submit the buffered message? Or lose it? Loss is
        not a big deal here.

"""


import os

import rospy
from std_msgs.msg import String

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_message_type_from_string
from lg_stats.msg import Event
import submitter


ROS_NODE_NAME = "lg_stats"
LG_STATS_DEBUG_TOPIC_DEFAULT = "debug"


class Processor(object):
    """
    Message processing is triggered by the reception of a next one.
    Didn't want any background threads or processes at this stage.
    It's lightweight anyway.
    The last (before ROS node shutdown) message is currently lost.

    """
    def __init__(self,
                 watched_topic=None,
                 msg_slot=None,
                 watched_field_name=None,
                 debug_pub=None,
                 resolution=20,
                 influxdb_client=None):
        self.watched_topic = watched_topic
        self.watched_field_name = watched_field_name
        self.msg_slot = msg_slot
        self.debug_pub = debug_pub
        self.resolution = resolution  # seconds
        self.time_of_last_msg = None
        self.last_msg = None  # message from the source topic
        self.influxdb_client = influxdb_client

    def publish(self, out_msg):
        """
        Publish the stats message on the ROS topic.

        """
        self.debug_pub.publish(out_msg)

    def compare_messages(self, msg_1, msg_2):
        """
        Returns True if relevant field of the messages for
        this processor instance is equal.

        """
        w = self.watched_field_name
        return True if getattr(msg_1, w) == getattr(msg_2, w) else False

    def get_outbound_message(self, src_msg):
        """
        Returns out-bound message for the /lg_stats/debug topic
        based on the data from the source topic message (src_msg).

        """
        application = getattr(src_msg, "application", '')  # if it exists
        if msg_slot:
            watched_field_name = "%s.%s" % (msg_slot, self.watched_field_name)
            value = str(getattr(getattr(src_msg, msg_slot), self.watched_field_name))
        out_msg = Event(src_topic=self.watched_topic,
                        field_name=watched_field_name,
                        type="event",
                        application=application,
                        # value is always string, if source value is e.g.
                        # boolean, it's converted to a string here
                        value=value)
        return out_msg

    def process_previous(self, curr_msg):
        """
        From #126:
            make ROS node A remember last message and submit it to stats when
            state hasn't changed for more than stats_resolution parameter

        If either condition (duration is shorter or the value of the incoming
        message differs), is not met, then the previous message is discarded.
        if elapsed.to_sec() > self.resolution and self.compare_messages(self.last_msg, curr_msg):
         ... -> this way there won't ever be anything submitted for e.g. /director/scene
            since the same message is never sent twice ... need to re-think this ...
            for now, apply just the time resolution condition.

        """
        elapsed = rospy.Time.now() - self.time_of_last_msg
        if elapsed.to_sec() > self.resolution:
            out_msg = self.get_outbound_message(self.last_msg)
            influx_data = self.influxdb_client.get_data_for_influx(out_msg)
            out_msg.influx = str(influx_data)
            rospy.loginfo("Submitting to InfluxDB: '%s'" % influx_data)
            self.publish(out_msg)
            self.influxdb_client.write_stats(influx_data)

    def process(self, msg):
        """
        Processing of a message is in fact triggered by reception of the
        consecutive message. The current one is merely buffered.

        """
        m = "Processor received: '%s'" % msg
        rospy.loginfo(m)
        # check previous message
        if self.last_msg:
            self.process_previous(msg)
        self.last_msg, self.time_of_last_msg = msg, rospy.Time.now()


def get_influxdb_client():
    submitter_type = rospy.get_param("~submission_type", None)
    host = rospy.get_param("~host", None)
    port = rospy.get_param("~port", None)
    database = rospy.get_param("~database", None)
    if not submitter_type or not host or not port:
        raise RuntimeError("No InfluxDB connection details provided in the roslaunch configuration.")
    return getattr(submitter, submitter_type)(host=host, port=port, database=database)


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, Event, queue_size=3)
    # resolution implementation is global across all watched topics which
    # may, may not be desirable ; in other words to have time resolution
    # configurable per specified source topic processor instance
    resolution = rospy.get_param("~resolution", 2)
    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(rospy.get_param("~activity_sources"))
    influxdb_client = get_influxdb_client()
    processors = []
    for ss in stats_sources:
        # dynamic import based on package/message_class string representation
        msg_type_module = get_message_type_from_string(ss["msg_type"])
        p = Processor(watched_topic=ss["topic"],
                      msg_slot=ss["slot"],
                      watched_field_name=ss["strategy"],
                      debug_pub=debug_topic_pub,
                      resolution=resolution,
                      influxdb_client=influxdb_client)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (ss["topic"], msg_type_module))
        rospy.Subscriber(ss["topic"], msg_type_module, p.process, queue_size=3)
        processors.append(p)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
