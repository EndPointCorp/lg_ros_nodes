#!/usr/bin/env python3

import rospy
from lg_media import GstreamerPool
from lg_msg_defs.msg import AdhocMedias
from lg_media.srv import MediaAppsInfo
from lg_media import DirectorMediaBridge
from lg_media import SRV_QUERY, ROS_NODE_NAME
from lg_common.helpers import make_soft_relaunch_callback, handle_initial_state
from interactivespaces_msgs.msg import GenericMessage

DEFAULT_VIEWPORT = 'center'
MEDIA_TYPE = 'video'  # mplayer used video rather than browser_video type


def main():
    rospy.init_node('lg_media_service_mplayer', anonymous=True)
    viewport_name = rospy.get_param("~viewport", DEFAULT_VIEWPORT)

    if not viewport_name:
        rospy.logerr("Viewport is not set - exiting")
        exit(1)

    topic_name = "/{0}/{1}".format(ROS_NODE_NAME, viewport_name)
    mplayer_pool = GstreamerPool(viewport_name)
    make_soft_relaunch_callback(mplayer_pool.handle_soft_relaunch, groups=["media"])

    # Initialize mplayer pool on specified viewport
    rospy.Subscriber(topic_name, AdhocMedias, mplayer_pool.handle_ros_message)

    # Initialize director => mplayer pool bridge
    adhoc_media_mplayer_pool_publisher = rospy.Publisher(topic_name,
                                                         AdhocMedias,
                                                         queue_size=3)
    adhoc_media_mplayer_director_bridge = DirectorMediaBridge(
        adhoc_media_mplayer_pool_publisher,
        viewport_name,
        MEDIA_TYPE)
    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     adhoc_media_mplayer_director_bridge.translate_director)

    handle_initial_state(adhoc_media_mplayer_director_bridge.translate_director)

    # Create service service about the mplayer pool service
    rospy.Service(SRV_QUERY,
                  MediaAppsInfo,
                  mplayer_pool.get_media_apps_info)
    rospy.spin()


if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
