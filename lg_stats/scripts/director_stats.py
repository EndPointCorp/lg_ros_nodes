#!/usr/bin/env python3

import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_stats import StatsHandler
from std_msgs.msg import Bool


def main():
    rospy.init_node('director_stats')

    print("helloo")
    stats_handler = StatsHandler()

    rospy.Subscriber('/director/scene', GenericMessage, stats_handler.handle_director)
    rospy.Subscriber('/activity/active', Bool, stats_handler.handle_activity)
    rospy.spin()


if __name__ == '__main__':
    main()
