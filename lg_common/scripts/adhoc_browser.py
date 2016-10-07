#!/usr/bin/env python

import rospy
from lg_common import AdhocBrowserPool
from lg_common.msg import AdhocBrowsers
from lg_common import AdhocBrowserDirectorBridge
from lg_common.helpers import make_soft_relaunch_callback, handle_initial_state
from lg_common.helpers import check_www_dependency
from interactivespaces_msgs.msg import GenericMessage
from lg_common.msg import Ready


def main():
    rospy.init_node('lg_adhoc_browser', anonymous=True)

    extensions_root = rospy.get_param('~extensions_root', '/opt/endpoint/chrome/extensions/')
    viewport_name = rospy.get_param('~viewport', None)
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    rosbridge_host = rospy.get_param('~rosbridge_port', 'localhost')
    depend_on_rosbridge = rospy.get_param('~depend_on_rosbridge', True)
    global_dependency_timeout = rospy.get_param('/global_dependency_timeout', 15)

    if not viewport_name:
        rospy.logerr("Viewport is not set in the roslaunch file. Exiting.")
        exit(1)

    """
    Initialize adhoc browser pool
    """

    topic_name = '/browser_service/{}'.format(viewport_name)
    common_topic_name = '/browser_service/browsers'

    adhocbrowser_pool = AdhocBrowserPool(viewport_name=viewport_name,
                                         extensions_root=extensions_root)
    make_soft_relaunch_callback(adhocbrowser_pool.handle_soft_relaunch,
                                groups=["media"])
    rospy.Subscriber(topic_name,
                     AdhocBrowsers,
                     adhocbrowser_pool.handle_ros_message)

    """
    Initialize director => browser pool bridge that translates director GenericMessage to AdhocBrowsers.msg
    """

    adhocbrowser_viewport_publisher = rospy.Publisher(
        topic_name, AdhocBrowsers, queue_size=3)

    adhocbrowser_aggregate_topic_publisher = rospy.Publisher(common_topic_name,
                                                             AdhocBrowsers,
                                                             queue_size=3)

    adhocbrowser_director_bridge = AdhocBrowserDirectorBridge(
        adhocbrowser_aggregate_topic_publisher,
        adhocbrowser_viewport_publisher,
        viewport_name)

    rospy.Subscriber('/director/scene', GenericMessage, adhocbrowser_director_bridge.translate_director)
    rospy.Subscriber('/director/ready', Ready, adhocbrowser_pool.unhide_browsers)

    rospy.wait_for_service('/readiness_node/ready', 15)
    handle_initial_state(adhocbrowser_director_bridge.translate_director)

    """
    Spin FTW
    """
    rospy.spin()

if __name__ == "__main__":
    main()
