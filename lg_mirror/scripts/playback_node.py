#!/usr/bin/env python

import os
import rospy
from lg_mirror.playback import MirrorPlaybackPool
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state


def main():
    rospy.init_node('mirror_playback')

    viewport = rospy.get_param('~viewport')
    if viewport is None:
        raise ValueError('Private parameter "viewport" is required')

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    pool = MirrorPlaybackPool(display, viewport)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     pool.handle_scene_msg)

    handle_initial_state(pool.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
