#!/usr/bin/env python

import rospy
from lg_common.msg import AdhocBrowsers
from lg_common import ReadinessNode
from std_msgs.msg import String
from lg_common.srv import NodeReady
from lg_common.msg import Ready
from lg_common.helpers import handle_initial_state
from interactivespaces_msgs.msg import GenericMessage


def main():
    rospy.init_node('lg_readiness')
    common_topic_name = '/browser_service/browsers'
    readiness_topic_name = '/director/ready'
    window_instances_topic_name = '/director/window/ready'

    readiness_publisher = rospy.Publisher(readiness_topic_name,
                                          Ready,
                                          queue_size=20)


    readiness_node = ReadinessNode(readiness_publisher)


    handle_initial_state(readiness_node.save_uscs_message)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     readiness_node.save_uscs_message)

    rospy.Subscriber(window_instances_topic_name,
                     String,
                     readiness_node.handle_readiness)

    rospy.Subscriber(common_topic_name,
                     AdhocBrowsers,
                     readiness_node.aggregate_browser_instances)

    rospy.Service('/readiness_node/ready', NodeReady, readiness_node.node_ready)

    rospy.spin()

if __name__ == "__main__":
    main()
