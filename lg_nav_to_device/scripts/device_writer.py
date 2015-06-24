#!/usr/bin/env python

import rospy
from lg_nav_to_device import DeviceWriter

def main():
    rospy.init_node('lg_nav_to_device')

    device_writer = DeviceWriter()
    sub = rospy.Subscriber('/spacenav/twist', Twist, device_writer.make_event)

    rospy.spin()

if __name__ == '__main__':
    main()
