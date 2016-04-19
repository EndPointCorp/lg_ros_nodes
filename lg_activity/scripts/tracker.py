#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from lg_activity import ActivitySource
from lg_activity import ActivityTracker
from lg_activity.srv import ActivityStates
from lg_activity import ActivitySourceDetector
from lg_common.helpers import get_params


def main():
    rospy.init_node('lg_activity', anonymous=False)

    activity_timeout = get_params('~activity_timeout', 120)
    activity_topic = get_params('~activity_publisher_topic', '/activity/active')
    sources_string = get_params('~activity_sources', '')
    memory_limit = get_params('~memory_limit', 102400)

    activity_publisher = rospy.Publisher(activity_topic, Bool, queue_size=1, latch=True)
    activity_sources = ActivitySourceDetector(sources_string).get_sources()

    activity_tracker = ActivityTracker(publisher=activity_publisher,
                                       timeout=activity_timeout,
                                       sources=activity_sources)

    activity_state_service = rospy.Service(activity_topic, ActivityStates, activity_tracker._get_state)

    rospy.spin()

if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
