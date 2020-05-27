#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from os import system
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'command_handler_node'


def handle_command(msg):
    code = msg.data
    system('lg-code-to-command -c ~/etc/keywatch.conf %s' % code)


def main():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber('/command_handler', String, handle_command)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
