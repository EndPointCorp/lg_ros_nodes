#!/usr/bin/env python

import os

import rospy
from std_msgs.msg import String

from interactivespaces_msgs.msg import GenericMessage
from lg_stats import ROS_NODE_NAME
from lg_stats import LG_STATS_DEBUG_TOPIC
from lg_stats import Processor
from lg_stats.msg import Stats


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic_pub = rospy.Publisher(LG_STATS_DEBUG_TOPIC, Stats, queue_size=3)

    # TODO
    # would like to roslaunch parametrize topic name, msg_type and watched field name
    # but roslaunch XML format 1) doesn't support lists as parameter type and doesn't
    # ...
    watched_topics = [dict(topic_name="/director/scene",
                           msg_type=GenericMessage,
                           watched_field_name="message")]
    processors = []
    for wt in watched_topics:
        topic_name, msg_type, watched_field_name = wt["topic_name"], wt["msg_type"], wt["watched_field_name"]
        p = Processor(watched_topic=topic_name,
                      watched_field_name=watched_field_name,
                      debug_pub=debug_topic_pub)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (topic_name, msg_type))
        rospy.Subscriber(topic_name, msg_type, p.process, queue_size=3)
        processors.append(p)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()

if __name__ == "__main__":
    main()