#!/usr/bin/env python3

import rospy
from lg_msg_defs.msg import ApplicationState
from std_msgs.msg import String


class NavMode:
    def __init__(self, mode_pub, default_mode):
        self.mode_pub = mode_pub
        self.mode = default_mode
        self.publish_mode()

    def handle_state(self, msg, mode):
        if msg.state != ApplicationState.VISIBLE:
            return
        self.mode = mode
        self.publish_mode()

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))


if __name__ == '__main__':
    rospy.init_node('nav_mode')

    default_mode = rospy.get_param('~default_mode')

    mode_pub = rospy.Publisher('/lg_navlib/nav_mode', String, latch=True, queue_size=1)
    nav_mode = NavMode(mode_pub, default_mode)

    modes = rospy.get_param(f'~modes')
    for mode, topics in modes.items():
        topics = [t.strip() for t in topics.split(',')]
        for topic in topics:
            rospy.Subscriber(topic, ApplicationState, nav_mode.handle_state, mode)

    rospy.spin()
