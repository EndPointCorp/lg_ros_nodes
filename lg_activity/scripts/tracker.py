#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from lg_activity import ActivitySource
from lg_activity import ActivityTracker
from lg_msg_defs.srv import ActivityStates
from lg_msg_defs.srv import Active
from lg_activity import ActivitySourceDetector
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'lg_activity'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


def main():
    rospy.init_node(NODE_NAME, anonymous=False)

    activity_timeout = rospy.get_param('~activity_timeout', 120)
    stats_activity_timeout = rospy.get_param('~stats_activity_timeout', 5)
    activity_topic = rospy.get_param('~activity_publisher_topic', '/activity/active')
    stats_activity_topic = rospy.get_param('~stats_activity_publisher_topic', '/activity/stats_active')
    activity_status_topic = rospy.get_param('~activity_publisher_topic', '/activity/status')
    sources_string = rospy.get_param('~activity_sources', None)
    memory_limit = rospy.get_param('~memory_limit', 102400)

    if not sources_string:
        logger.error('No or blank source string supplied, exiting...')
        return

    activity_publisher = rospy.Publisher(activity_topic, Bool, queue_size=1, latch=True)
    stats_activity_publisher = rospy.Publisher(stats_activity_topic, Bool, queue_size=1, latch=True)
    activity_sources = ActivitySourceDetector(sources_string).get_sources()

    activity_tracker = ActivityTracker(publisher=activity_publisher,
                                       stats_activity_publisher=stats_activity_publisher,
                                       timeout=activity_timeout,
                                       stats_activity_timeout=stats_activity_timeout,
                                       sources=activity_sources)

    rospy.Service(activity_topic, ActivityStates, activity_tracker._get_state)
    rospy.Service(activity_status_topic, Active, activity_tracker._get_activity_status)

    while not rospy.is_shutdown():
        activity_tracker.poll_activities()
        rospy.sleep(1)


if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
