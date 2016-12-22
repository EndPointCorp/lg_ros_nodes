#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from lg_nav_to_device import DeviceWriter
from lg_common.msg import ApplicationState
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'lg_nav_to_device'


def main():
    rospy.init_node(NODE_NAME)

    scale = rospy.get_param('~scale', 512.0)

    device_writer = DeviceWriter(scale)
    sub = rospy.Subscriber('/spacenav_wrapper/twist', Twist, device_writer.make_event)
    rospy.Subscriber('/earth/state', ApplicationState, device_writer.set_state)

    rospy.spin()

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
