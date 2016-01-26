#!/usr/bin/env python
"""
NOTES:

- Session.msg
    borrowed from statistics/msg/Session.msg
    it's possible to introduce this dependency (or leave it as is)

- check statistics/StatsD.msg

Stats.msg
    type - event type or time series type
    value - value associated with event OR rate associated with series type
    identification: system_name + application (+ src_topic)

Will need to define association between source topics and type of
    result statistical data (whether it's event based or time-series based).
    This will need to be either configured (not sure if "output-type" will
    be easy to blend with activity_sources processing. Or hard-coded
    internal relational matrix (output msg fields: type, application).

"""


import os

import rospy
from std_msgs.msg import String

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_message_type_from_string
from lg_stats.msg import Stats


ROS_NODE_NAME = "lg_stats"
LG_STATS_DEBUG_TOPIC_DEFAULT = "debug"
SYSTEM_NAME = os.uname()[1]


class Processor(object):
    def __init__(self,
                 watched_topic=None,
                 watched_field_name=None,
                 debug_pub=None):
        self.watched_topic = watched_topic
        self.watched_field_name = watched_field_name
        self.debug_pub = debug_pub

    def process(self, msg):
        m = "processor received: '%s'" % msg
        rospy.loginfo(m)

        # TODO
        # later the time resolution will come into play
        # so that messages of the kind will be buffered
        # and a change into influx submitted only upon
        # changing the message after a certain time interval
        # need to think up message change detection

        stats_msg = Stats(system_name=SYSTEM_NAME,
                          src_topic=self.watched_topic,
                          field_name=self.watched_field_name,
                          # value is always string, if source value is e.g.
                          # boolean, it's converted to a string here
                          value=str(getattr(msg, self.watched_field_name)))
        self.debug_pub.publish(stats_msg)

        # TODO
        # once determined that stats message shall be send out
        # send it via connecting to the influxdb db telegraf agent


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, Stats, queue_size=3)

    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(rospy.get_param("~activity_sources"))

    processors = []
    for ss in stats_sources:
        # dynamic import based on package/message_class string representation
        msg_type_module = get_message_type_from_string(ss["msg_type"])
        p = Processor(watched_topic=ss["topic"],
                      watched_field_name=ss["strategy"],
                      debug_pub=debug_topic_pub)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (ss["topic"], msg_type_module))
        rospy.Subscriber(ss["topic"], msg_type_module, p.process, queue_size=3)
        processors.append(p)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
