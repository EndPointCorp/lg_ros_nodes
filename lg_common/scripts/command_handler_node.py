#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from os import system


def handle_command(msg):
    code = msg.data
    system('lg-code-to-command -c ~/etc/keywatch.conf %s' % code)


def main():
    rospy.init_node('command_handler_node')
    rospy.Subscriber('/command_handler', String, handle_command)

    rospy.spin()

if __name__ == '__main__':
    main()
