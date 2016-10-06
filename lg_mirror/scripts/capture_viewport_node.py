#!/usr/bin/env python

import os
import rospy
from lg_mirror.capture_viewport import CaptureViewport
from lg_mirror.utils import get_viewport_topic
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state
from sensor_msgs.msg import Image


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

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    show_pointer = str(rospy.get_param('~show_pointer', False)).lower()
    framerate = int(rospy.get_param('~framerate', 30))

    image_topic = get_viewport_topic(viewport)
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)

    capture = CaptureViewport(viewport,
                              display,
                              show_pointer,
                              framerate,
                              image_pub)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     capture.handle_scene_msg)

    handle_initial_state(capture.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    main()


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
