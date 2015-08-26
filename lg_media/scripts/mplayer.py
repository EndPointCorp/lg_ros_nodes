#!/usr/bin/env python

import rospy
from lg_media import MplayerPool
from lg_media.msg import AdhocMedias
from lg_media.srv import MediaAppsInfo

DEFAULT_VIEWPORT='center'

def main():
    rospy.init_node('lg_media_service_mplayer', log_level=rospy.DEBUG)
    viewport_name = rospy.get_param("~viewport", DEFAULT_VIEWPORT)

    if not viewport_name:
        rospy.logerr("Viewport is not set - exiting")
        exit(1)

    media_service = MediaService()
    topic_name = "/media_service/{}".format(viewport_name)
    mplayer_pool = MplayerPool(viewport_name)

    """
    Initialize mplayer pool on specified viewport
    """
    rospy.Subscriber(topic_name, AdhocMedias, mplayer_pool.handle_ros_message)

    """
    Initialize director => mplayer pool bridge
    """

    adhoc_media_mplayer_pool = rospy.Publisher(
            topic_name, AdhocMedias, queue_size=3
        )

    rospy.spin()


if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
