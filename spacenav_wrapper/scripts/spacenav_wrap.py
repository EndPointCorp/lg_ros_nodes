#!/usr/bin/env python

import rospy
from spacenav_wrapper import SpacenavWrapper
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def get_fullscale():
    params = rospy.get_param_names()
    for param in params:
        if 'spacenav' in param and 'full_scale' in param:
            return int(rospy.get_param(param))
    # assuming no full_scale since there wasn't a spacenav node with
    # full_scale as a parameter
    return 0


def main():
    rospy.init_node('spacenav_wrapper')
    topic_root = rospy.get_param('~root_topic', '/spacenav_wrapper')
    gutter_val = rospy.get_param('~gutter_value', 0.0)
    buffer_size = rospy.get_param('~buffer_size', 200)
    should_relaunch = rospy.get_param('~relaunch', False)
    if topic_root[0] != '/':
        topic_root = '/' + topic_root
    twist = rospy.Publisher(topic_root + '/twist', Twist, queue_size=10)
    joy = rospy.Publisher(topic_root + '/joy', Joy, queue_size=10)
    rezero_pub = rospy.Publisher('/rezero', String, queue_size=10)
    full_scale = get_fullscale()

    def rezero():
        if should_relaunch:
            rezero_pub.publish('rezero')
    s = SpacenavWrapper(twist=twist, joy=joy, rezero=rezero,
                        full_scale=full_scale, gutter_val=gutter_val,
                        buffer_size=buffer_size)

    rospy.Subscriber('/spacenav/twist', Twist, s.handle_twist)
    rospy.Subscriber('/spacenav/joy', Joy, s.handle_joy)

    rospy.spin()

if __name__ == '__main__':
    main()
