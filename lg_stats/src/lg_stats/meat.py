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
    be easy to blend with activity_sources processing. Or hard-coded in an
    internal relational matrix (output msg fields: type, application).

TODO:
    - implement basic unittests for methods of Processor

    - upon node shutdown - submit the buffered message? Or lose it? Loss is
        not a big deal here.

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
                 resolution=20):
        self.watched_topic = watched_topic
        self.watched_field_name = watched_field_name
        self.debug_pub = debug_pub
        self.resolution = resolution  # seconds
        self.time_of_last_msg = None
        self.last_msg = None

    def publish(self, msg):
        """
        Publish the stats message on the ROS topic.

        TODO:
            stats submission (via telegraf submission influxdb agent) here

        """
        stats_msg = Stats(system_name=SYSTEM_NAME,
                          src_topic=self.watched_topic,
                          field_name=self.watched_field_name,
                          # value is always string, if source value is e.g.
                          # boolean, it's converted to a string here
                          value=str(getattr(msg, self.watched_field_name)))
        self.debug_pub.publish(stats_msg)

    def compare_messages(self, msg_1, msg_2):
        """
        Returns True if relevant field of the messages for
        this processor instance is equal.

        """
        w = self.watched_field_name
        return True if getattr(msg_1, w) == getattr(msg_2, w) else False

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
            self.publish(self.last_msg)

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


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, Stats, queue_size=3)
    # resolution implementation is global across all watched topics which
    # may, may not be desirable ; in other words to have time resolution
    # configurable per specified source topic processor instance
    resolution = rospy.get_param("~resolution", 2)
    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(rospy.get_param("~activity_sources"))
    processors = []
    for ss in stats_sources:
        # dynamic import based on package/message_class string representation
        msg_type_module = get_message_type_from_string(ss["msg_type"])
        p = Processor(watched_topic=ss["topic"],
                      watched_field_name=ss["strategy"],
                      debug_pub=debug_topic_pub,
                      resolution=resolution)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (ss["topic"], msg_type_module))
        rospy.Subscriber(ss["topic"], msg_type_module, p.process, queue_size=3)
        processors.append(p)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
