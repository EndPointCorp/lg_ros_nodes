#!/usr/bin/env python3

import os
import rospy
from lg_mirror.playback import MirrorPlaybackPool
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state
from lg_common.helpers import run_with_influx_exception_handler
from lg_common.helpers import required_param


NODE_NAME = 'mirror_playback'


def main():
    rospy.init_node(NODE_NAME)

    viewport = required_param('~viewport')
    janus_url = required_param('/lg_mirror/janus_rest_uri')

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    pool = MirrorPlaybackPool(display, viewport, janus_url)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     pool.handle_scene_msg)

    handle_initial_state(pool.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
