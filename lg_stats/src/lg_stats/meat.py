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

from influxdb import InfluxDBClient
import rospy
from std_msgs.msg import String

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_message_type_from_string
from lg_stats.msg import Event


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
                 watched_field_name=None,
                 debug_pub=None,
                 resolution=20,
                 influxdb_client=None):
        self.watched_topic = watched_topic
        self.watched_field_name = watched_field_name
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

    def get_outbound_message_and_influx_dict(self, src_msg):
        """
        Returns out-bound message for the /lg_stats/debug topic
        based on the data from the source topic message (src_msg).
        And Python dictionary for later InfluxDB submission.

        """
        application = getattr(src_msg, "application", None)  # if it exists
        out_msg = Event(src_topic=self.watched_topic,
                        field_name=self.watched_field_name,
                        type="event",
                        application=application,
                        # value is always string, if source value is e.g.
                        # boolean, it's converted to a string here
                        value=str(getattr(src_msg, self.watched_field_name)))
        # python representation of the InfluxDB default line_protocol
        influx_dict = dict(measurement=out_msg.src_topic,
                           tags=dict(application=out_msg.application,
                                     field_name=out_msg.field_name,
                                     type=out_msg.type,
                                     value=out_msg.value),
                           # timestamp may be added here or will be added by the server
                           #"time": "2015-11-10T23:00:00Z",
                           # fields must be of type float
                           fields=dict(value=0.0))
        out_msg.influx = str(influx_dict)
        return out_msg, influx_dict

    def write_to_influx(self, influx_dict):
        """
        Submit data to influx (via locally running telegraf agent).
        The Python Influx library converts the Python dictionary to
        the default *line_protocol* before submitting to Influx.

        """
        if self.influxdb_client:
            self.influxdb_client.write_points([influx_dict])

    def process_previous(self, curr_msg):
        """
        From #126:
            make ROS node A remember last message and submit it to stats when
            state hasn't changed for more than stats_resolution parameter

        If either condition (duration is shorter or the value of the incoming
        message differs), is not met, then the previous message is discarded.

        """
        elapsed = rospy.Time.now() - self.time_of_last_msg
        if elapsed.to_sec() > self.resolution and self.compare_messages(self.last_msg, curr_msg):
            out_msg, influx_dict = self.get_outbound_message_and_influx_dict(self.last_msg)
            self.publish(out_msg)
            self.write_to_influx(influx_dict)

    def process(self, msg):
        """
        Processing of a message is in fact triggered by reception of the
        consecutive message. The current one is merely buffered.

        """
        m = "processor received: '%s'" % msg
        rospy.loginfo(m)
        # check previous message
        if self.last_msg:
            self.process_previous(msg)
        self.last_msg, self.time_of_last_msg = msg, rospy.Time.now()


def get_influxdb_client():
    telegraf_host = rospy.get_param("~telegraf_host", None)
    telegraf_port = rospy.get_param("~telegraf_port", None)
    influxdb_database = rospy.get_param("~influxdb_database", None)
    if telegraf_host and telegraf_port and influxdb_database:
        influxdb_client = InfluxDBClient(host=telegraf_host,
                                         port=telegraf_port,
                                         database=influxdb_database)
    else:
        influxdb_client = None
    return influxdb_client


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
                      watched_field_name=ss["strategy"],
                      debug_pub=debug_topic_pub,
                      resolution=resolution,
                      influxdb_client=influxdb_client)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (ss["topic"], msg_type_module))
        rospy.Subscriber(ss["topic"], msg_type_module, p.process, queue_size=3)
        processors.append(p)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
