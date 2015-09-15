#!/usr/bin/env python

import rospy
from lg_media.msg import AdhocMedias
from lg_media import DirectorMediaBridge
from interactivespaces_msgs.msg import GenericMessage

DEFAULT_VIEWPORT = 'center'
MEDIA_TYPE = 'browser_video'
VIDEOSYNC_URL = 'http://lg-head/lg_sv/webapps/videosync'


def main():
    rospy.init_node('lg_media_service_browser_player', anonymous=True)
    viewport_name = rospy.get_param('~viewport', DEFAULT_VIEWPORT)

    topic_name = '/media_service/%s' % viewport_name

    adhoc_media_publisher = rospy.Publisher(topic_name, AdhocMedias,
                                            queue_size=3)
    director_bridge = DirectorMediaBridge(adhoc_media_publisher, viewport_name, MEDIA_TYPE)
    rospy.Subscriber('/director/scene', GenericMessage,
                     director_bridge.translate_director)
    rospy.spin()

if __name__ == '__main__':
    main()
