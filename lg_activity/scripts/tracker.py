#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from lg_activity import ActivitySource
from lg_activity import ActivityTracker
from lg_activity.srv import ActivityStates
from lg_activity import ActivitySourceDetector


def main():
    rospy.init_node('lg_activity', anonymous=False)

    activity_timeout = rospy.get_param('~activity_timeout', 120)
    activity_topic = rospy.get_param('~activity_publisher_topic', '/activity/active')
    sources_string = rospy.get_param('~activity_sources', '')
    memory_limit = rospy.get_param('~memory_limit', 102400)

    activity_publisher = rospy.Publisher(activity_topic, Bool, queue_size=1, latch=True)
    activity_sources = ActivitySourceDetector(sources_string).get_sources()

    activity_tracker = ActivityTracker(publisher=activity_publisher,
                                       timeout=activity_timeout,
                                       sources=activity_sources)

    activity_state_service = rospy.Service(activity_topic, ActivityStates, activity_tracker._get_state)

    while not rospy.is_shutdown():
        activity_tracker.poll_activities()
        rospy.sleep(1)

if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
