#!/usr/bin/env python

import os
import rospy
from lg_mirror.capture_viewport import CaptureViewport
from lg_mirror.utils import viewport_to_multicast_group
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state, required_param
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'mirror_capture_viewport'


def main():
    rospy.init_node(NODE_NAME)

    viewport = required_param('~viewport')
    show_pointer = str(rospy.get_param('~show_pointer', False)).lower()
    framerate = int(rospy.get_param('~framerate', 30))
    janus_host = required_param('/lg_mirror/janus_stream_host', str)
    janus_port = required_param('~janus_port', int)

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    capture = CaptureViewport(viewport,
                              display,
                              show_pointer,
                              framerate,
                              janus_host,
                              janus_port)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     capture.handle_scene_msg)

    handle_initial_state(capture.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
