#!/usr/bin/env python

import rospy
from lg_media import MplayerPool
from lg_media.msg import AdhocMedias
from lg_media.srv import MediaAppsInfo
from lg_media import DirectorMediaBridge
from interactivespaces_msgs.msg import GenericMessage

DEFAULT_VIEWPORT='center'

def main():
    rospy.init_node('lg_media_service_mplayer', anonymous=True, log_level=rospy.DEBUG)
    viewport_name = rospy.get_param("~viewport", DEFAULT_VIEWPORT)

    if not viewport_name:
        rospy.logerr("Viewport is not set - exiting")
        exit(1)

    topic_name = "/media_service/{}".format(viewport_name)
    mplayer_pool = MplayerPool(viewport_name)

    """
    Initialize mplayer pool on specified viewport
    """
    rospy.Subscriber(topic_name, AdhocMedias, mplayer_pool.handle_ros_message)

    """
    Initialize director => mplayer pool bridge
    """

    adhoc_media_mplayer_pool_publisher = rospy.Publisher(
            topic_name, AdhocMedias, queue_size=3
        )

    adhoc_media_mplayer_director_bridge = DirectorMediaBridge(adhoc_media_mplayer_pool_publisher, viewport_name)

    rospy.Subscriber('/director/scene', GenericMessage, adhoc_media_mplayer_director_bridge.translate_director)

    rospy.spin()


if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
