#!/usr/bin/env python

import rospy
from lg_common import AdhocBrowserPool
from lg_common.msg import AdhocBrowsers
from lg_common import AdhocBrowserDirectorBridge
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import get_params


def main():
    rospy.init_node('lg_adhoc_browser', anonymous=True)

    viewport_name = get_params('~viewport', None)
    browser_binary = get_params('~browser_binary', '/usr/bin/google-chrome')

    if not viewport_name:
        rospy.logerr("Viewport is not set in the roslaunch file. Exiting.")
        exit(1)

    """
    Initialize adhoc browser pool
    """
    topic_name = '/browser_service/{}'.format(viewport_name)
    adhocbrowser_pool = AdhocBrowserPool(viewport_name)
    rospy.Subscriber(topic_name, AdhocBrowsers, adhocbrowser_pool.handle_ros_message)

    """
    Initialize director => browser pool bridge that translates director GenericMessage to AdhocBrowsers.msg
    """

    adhocbrowser_director_bridge_publisher = rospy.Publisher(
        topic_name, AdhocBrowsers, queue_size=3)

    adhocbrowser_director_bridge = AdhocBrowserDirectorBridge(adhocbrowser_director_bridge_publisher, viewport_name)

    rospy.Subscriber('/director/scene', GenericMessage, adhocbrowser_director_bridge.translate_director)

    """
    Spin FTW
    """
    rospy.spin()

if __name__ == "__main__":
    main()
