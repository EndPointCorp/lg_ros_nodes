#!/usr/bin/env python
"""
TODO: Browser is not changing url yet.
"""

import rospy
from lg_common.msg import AdhocBrowsers
from lg_common import AdhocBrowserPool


def main():
    rospy.init_node('lg_adhoc_browser', anonymous=True)

    vieport_name = rospy.get_param('~viewport', None)

    if not vieport_name:
        rospy.logerr("Viewport is not set in the roslaunch file. Exiting.")
        exit(1)

    topic_name = '/browser_service/{}'.format(vieport_name)
    client = AdhocBrowserPool()

    rospy.Subscriber(topic_name, AdhocBrowsers, client.handle_ros_message)

    rospy.spin()


if __name__ == "__main__":
    main()
