#!/usr/bin/env python

import os
import rospy
from lg_mirror.capture_viewport import CaptureViewport
from lg_mirror.utils import viewport_to_multicast_group
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state


def required_param(key, coer=None):
    val = rospy.get_param(key)
    if val is None:
        raise ValueError('"{}" param required'.format(key))
    if coer is not None:
        val = coer(val)
    return val


def main():
    rospy.init_node('mirror_capture_viewport')

    viewport = required_param('~viewport')
    janus_host = required_param('/lg_mirror/janus_stream_host', str)
    janus_port = required_param('~janus_port', int)

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    show_pointer = str(rospy.get_param('~show_pointer', False)).lower()

    capture = CaptureViewport(viewport,
                              display,
                              show_pointer,
                              janus_host,
                              janus_port)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     capture.handle_scene_msg)

    handle_initial_state(capture.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    main()


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
