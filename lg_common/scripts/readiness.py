#!/usr/bin/env python3

import rospy
from lg_common.msg import AdhocBrowsers
from lg_common import ReadinessNode
from lg_common import ReadinessHandbrake
from std_msgs.msg import String
from lg_common.srv import NodeReady
from lg_common.msg import Ready
from lg_common.helpers import handle_initial_state
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'readiness_node'


def main():
    rospy.init_node(NODE_NAME)
    common_topic_name = '/browser_service/browsers'
    readiness_topic_name = '/director/ready'
    window_instances_topic_name = '/director/window/ready'

    readiness_timeout = rospy.get_param("/readiness/timeout", 10)

    readiness_publisher = rospy.Publisher(readiness_topic_name,
                                          Ready,
                                          queue_size=20)

    timeout_publisher = rospy.Publisher('/director/window/error',
                                        String,
                                        queue_size=10)

    readiness_node = ReadinessNode(
        readiness_publisher=readiness_publisher,
        timeout_publisher=timeout_publisher
    )

    readiness_handbrake = ReadinessHandbrake(
        callback=readiness_node.try_to_become_ready,
        timeout=readiness_timeout,
    )

    handle_initial_state(readiness_node.save_uscs_message)
    handle_initial_state(readiness_handbrake.handle_uscs_message)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     readiness_node.save_uscs_message)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     readiness_handbrake.handle_uscs_message)

    rospy.Subscriber(window_instances_topic_name,
                     String,
                     readiness_node.handle_readiness)

    rospy.Subscriber(common_topic_name,
                     AdhocBrowsers,
                     readiness_node.aggregate_browser_instances)

    rospy.Service('/readiness_node/ready', NodeReady, readiness_node.node_ready)

    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
