#!/usr/bin/env python3

import re
import rospy
from std_msgs.msg import String
from lg_common import AdhocBrowserPool
from lg_msg_defs.msg import AdhocBrowsers
from lg_common import AdhocBrowserDirectorBridge
from lg_common.helpers import make_soft_relaunch_callback, handle_initial_state
from lg_common.helpers import wait_for_pub_sub_connections
from lg_common.helpers import run_with_influx_exception_handler
from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.msg import Ready

NODE_NAME = 'lg_adhoc_browser'


def main():
    rospy.init_node(NODE_NAME, anonymous=True)

    extensions_root = rospy.get_param('~extensions_root', '/opt/endpoint/chrome/extensions/')
    viewport_name = rospy.get_param('~viewport', None)
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    rosbridge_host = rospy.get_param('~rosbridge_port', 'localhost')
    depend_on_rosbridge = rospy.get_param('~depend_on_rosbridge', True)
    global_dependency_timeout = rospy.get_param('/global_dependency_timeout', 15)
    hide_delay = rospy.get_param('~hide_delay', 0.5)
    destroy_delay = rospy.get_param('~destroy_delay', 2)

    if not viewport_name:
        rospy.logerr("Viewport is not set in the roslaunch file. Exiting.")
        exit(1)

    """
    Initialize adhoc browser pool
    """

    topic_name = '/browser_service/{}'.format(viewport_name)
    common_topic_name = '/browser_service/browsers'

    actors = []

    adhocbrowser_pool = AdhocBrowserPool(viewport_name=viewport_name,
                                         extensions_root=extensions_root,
                                         hide_delay=hide_delay,
                                         destroy_delay=destroy_delay)

    make_soft_relaunch_callback(adhocbrowser_pool.handle_soft_relaunch,
                                groups=["media"])

    browser_service_sub = rospy.Subscriber(
        topic_name,
        AdhocBrowsers,
        adhocbrowser_pool.handle_ros_message
    )

    actors.append(browser_service_sub)

    """
    Initialize director => browser pool bridge that translates director GenericMessage to AdhocBrowsers.msg
    """

    adhocbrowser_viewport_publisher = rospy.Publisher(
        topic_name, AdhocBrowsers, queue_size=3)
    actors.append(adhocbrowser_viewport_publisher)

    adhocbrowser_aggregate_topic_publisher = rospy.Publisher(common_topic_name,
                                                             AdhocBrowsers,
                                                             queue_size=3)
    actors.append(adhocbrowser_aggregate_topic_publisher)

    adhocbrowser_director_bridge = AdhocBrowserDirectorBridge(
        adhocbrowser_aggregate_topic_publisher,
        adhocbrowser_viewport_publisher,
        viewport_name)

    director_scene_sub = rospy.Subscriber('/director/scene', GenericMessage, adhocbrowser_director_bridge.translate_director)
    actors.append(director_scene_sub)
    director_ready_sub = rospy.Subscriber('/director/ready', Ready, adhocbrowser_pool.unhide_browsers)
    actors.append(director_ready_sub)

    rospy.wait_for_service('/readiness_node/ready', 15)
    rospy.wait_for_service('/lg_offliner/status', 15)
    wait_for_pub_sub_connections(actors)

    handle_initial_state(adhocbrowser_director_bridge.translate_director)

    """
    Initialize overlay hiding listener
    """
    def getBrowserIds(msg):
        s = msg.data
        if '[' in s and ']' in s:
            ids = [sp for sp in re.split('\[\]\, ', s) if len(sp) > 0]
            adhocbrowser_pool.minimize_browsers(ids)
        else:
            adhocbrowser_pool.minimize_browsers([s])

    rospy.Subscriber('/director/minimize', String, getBrowserIds)

    """
    Spin FTW
    """
    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
