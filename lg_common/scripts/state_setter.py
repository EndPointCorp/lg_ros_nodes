#!/usr/bin/env python3

import rospy
from lg_common import StateChanger
from lg_common.msg import StringArray
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'state_handler'


def main():
    rospy.init_node(NODE_NAME)

    state_changer = StateChanger()
    rospy.Subscriber('/state_handler/activate', StringArray, state_changer.locked_state_handler)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
