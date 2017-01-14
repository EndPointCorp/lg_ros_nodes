#!/usr/bin/env python

import rospy
from spacenav_wrapper import SpacenavWrapper
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'spacenav_wrapper'


def get_fullscale():
    params = rospy.get_param_names()
    for param in params:
        if 'spacenav' in param and 'full_scale' in param:
            return int(rospy.get_param(param))
    # assuming no full_scale since there wasn't a spacenav node with
    # full_scale as a parameter
    return 0


def main():
    rospy.init_node(NODE_NAME)
    topic_root = rospy.get_param('~root_topic', '/spacenav_wrapper')
    gutter_val = rospy.get_param('~gutter_value', 0.0)
    if topic_root[0] != '/':
        topic_root = '/' + topic_root
    twist = rospy.Publisher(topic_root + '/twist', Twist, queue_size=10)
    joy = rospy.Publisher(topic_root + '/joy', Joy, queue_size=10)
    full_scale = get_fullscale()

    s = SpacenavWrapper(twist=twist, joy=joy, gutter_val=gutter_val)

    rospy.Subscriber('/spacenav/twist', Twist, s.handle_twist)
    rospy.Subscriber('/spacenav/joy', Joy, s.handle_joy)

    rospy.spin()

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
