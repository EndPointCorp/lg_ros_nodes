#!/usr/bin/env python

import os
import rospy
from lg_mirror.capture_viewport import CaptureViewport
from lg_mirror.utils import get_viewport_image_topic
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state, required_param
from sensor_msgs.msg import CompressedImage
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'mirror_capture_viewport'


def main():
    rospy.init_node(NODE_NAME)

    viewport = required_param('~viewport')

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    show_pointer = str(rospy.get_param('~show_pointer', False)).lower()
    framerate = int(rospy.get_param('~framerate', 30))
    quality = int(rospy.get_param('~quality', 85))

    image_topic = get_viewport_image_topic(viewport)
    image_pub = rospy.Publisher(image_topic, CompressedImage, queue_size=1)

    capture = CaptureViewport(viewport,
                              display,
                              show_pointer,
                              framerate,
                              quality,
                              image_pub)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     capture.handle_scene_msg)

    handle_initial_state(capture.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
