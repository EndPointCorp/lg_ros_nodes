#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from lg_nav_to_device import DeviceWriter


def main():
    rospy.init_node('lg_nav_to_device')

    scale = rospy.get_param('~scale', 512.0)

    device_writer = DeviceWriter(scale)
    sub = rospy.Subscriber('/spacenav/twist', Twist, device_writer.make_event)

    rospy.spin()

if __name__ == '__main__':
    main()
