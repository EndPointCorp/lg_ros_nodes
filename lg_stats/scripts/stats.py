#!/usr/bin/env python

import os

import rospy
from std_msgs.msg import String

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_message_type_from_string
from lg_stats import ROS_NODE_NAME
from lg_stats import LG_STATS_DEBUG_TOPIC_DEFAULT
from lg_stats import Processor
from lg_stats.msg import Stats


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, Stats, queue_size=3)

    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(rospy.get_param("~activity_sources"))

    # TODO
    # should do checkes whether such a topic exists ...

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

if __name__ == "__main__":
    main()