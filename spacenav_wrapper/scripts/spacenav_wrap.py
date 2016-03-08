#!/usr/bin/env python

import rospy
from spacenav_wrapper import SpacenavWrapper
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

def main():
    rospy.init_node('spacenav_wrapper')
    topic_root = rospy.get_param('~root_topic', '/spacenav_wrapper')
    if topic_root[0] != '/':
        topic_root = '/' + topic_root
    twist = rospy.Publisher(topic_root + '/twist', Twist, queue_size=10)
    joy = rospy.Publisher(topic_root + '/joy', Joy, queue_size=10)
    command_handler = rospy.Publisher('/command_handler', String, queue_size=10)
    def rezero():
        command_handler.publish('rezero')
    s = SpacenavWrapper(twist=twist, joy=joy, rezero=rezero)

    rospy.Subscriber('/spacenav/twist', Twist, s.handle_twist)
    rospy.Subscriber('/spacenav/joy', Joy, s.handle_joy)

    rospy.spin()

if __name__ == '__main__':
    main()
