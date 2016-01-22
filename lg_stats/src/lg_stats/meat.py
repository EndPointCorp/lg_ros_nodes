#!/usr/bin/env python

import os

import rospy
from lg_stats.msg import Stats


ROS_NODE_NAME = "lg_stats"
LG_STATS_DEBUG_TOPIC = "%s/debug" % ROS_NODE_NAME
LG_SYSTEM_NAME = os.uname()[1]


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

        stats_msg = Stats(lg_system_name=LG_SYSTEM_NAME,
                          src_topic=self.watched_topic,
                          field_name=self.watched_field_name,
                          value=getattr(msg, self.watched_field_name))
        self.debug_pub.publish(stats_msg)

        # TODO
        # once determined that stats message shall be send out
        # send it via connecting to the influxdb db telegraf agent