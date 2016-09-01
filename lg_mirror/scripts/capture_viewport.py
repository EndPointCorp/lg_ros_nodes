#!/usr/bin/env python

import os
import rospy
from lg_mirror.capture_viewport import CaptureViewport
from lg_mirror.utils import viewport_to_multicast_group
from interactivespaces_msgs.msg import GenericMessage


def main():
    rospy.init_node('mirror_capture_viewport')

    viewport = rospy.get_param('~viewport')
    if viewport is None:
        raise ValueError('Private parameter "viewport" is required')

    env_display = os.environ.get('DISPLAY')
    display = rospy.get_param('~display', env_display)
    if display is None:
        raise ValueError('DISPLAY env or private "display" param required')

    quality = int(rospy.get_param('~quality', 85))
    show_pointer = str(rospy.get_param('~show_pointer', False)).lower()
    host = rospy.get_param('~host', viewport_to_multicast_group(viewport))

    capture = CaptureViewport(viewport,
                              display,
                              quality,
                              show_pointer,
                              host)

    rospy.Subscriber('/director/scene',
                     GenericMessage,
                     capture.handle_scene_msg)

    rospy.spin()


if __name__ == '__main__':
    main()


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
