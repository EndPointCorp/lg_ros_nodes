#!/usr/bin/env python3

import rospy
from lg_msg_defs.msg import AdhocMedias
from lg_media import DirectorMediaBridge
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state
from lg_common.helpers import run_with_influx_exception_handler

DEFAULT_VIEWPORT = 'center'
MEDIA_TYPE = 'browser_video'
VIDEOSYNC_URL = 'http://lg-head/lg_sv/webapps/videosync'
NODE_NAME = 'lg_media_service_browser_player'


def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    viewport_name = rospy.get_param('~viewport', DEFAULT_VIEWPORT)

    topic_name = '/media_service/browser/%s' % viewport_name

    adhoc_media_publisher = rospy.Publisher(topic_name, AdhocMedias,
                                            queue_size=3)
    director_bridge = DirectorMediaBridge(adhoc_media_publisher, viewport_name, MEDIA_TYPE)
    rospy.Subscriber('/director/scene', GenericMessage,
                     director_bridge.translate_director)
    handle_initial_state(director_bridge.translate_director)
    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
