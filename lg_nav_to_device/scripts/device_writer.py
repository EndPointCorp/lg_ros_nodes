#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from lg_nav_to_device import DeviceWriter
from lg_common.msg import ApplicationState


def main():
    rospy.init_node('lg_nav_to_device')

    scale = rospy.get_param('~scale', 512.0)

    device_writer = DeviceWriter(scale)
    sub = rospy.Subscriber('/spacenav/twist', Twist, device_writer.make_event)
    rospy.Subscriber('/earth/state', ApplicationState, device_writer.set_state)

    rospy.spin()

if __name__ == '__main__':
    main()
