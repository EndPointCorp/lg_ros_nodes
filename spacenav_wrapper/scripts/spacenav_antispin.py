#!/usr/bin/env python3

import rospy
from spacenav_wrapper import SpacenavRezeroer
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'spacenav_anti_spin'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


def get_fullscale():
    params = rospy.get_param_names()
    for param in params:
        if 'spacenav' in param and 'full_scale' in param:
            full_scale = int(rospy.get_param(param))
            if full_scale == 1:
                return 255
    # assuming no full_scale since there wasn't a spacenav node with
    # full_scale as a parameter
    return 0.68


def main():
    rospy.init_node(NODE_NAME)
    topic_root = rospy.get_param('~spacenav_topic', '/spacenav/twist')
    seconds_before_rezero = int(rospy.get_param('~seconds_before_rezero', 30))
    rate = int(rospy.get_param('~rate', 4))
    should_relaunch = rospy.get_param('~relaunch', True)

    rezero_pub = rospy.Publisher('/rezero', String, queue_size=10)

    def rezero():
        if should_relaunch:
            rezero_pub.publish('rezero')
        else:
            logger.warning('A rezero is needed, but the rezeroer dose not have permission')

    threshold = get_fullscale() * 0.0147  # this can be tweaked

    rezeroer = SpacenavRezeroer(threshold=threshold,
                                seconds_before_rezero=seconds_before_rezero,
                                rate=rate, rezero=rezero)

    rospy.Subscriber(topic_root, Twist, rezeroer.on_spacenav)

    rospy.Timer(rospy.Duration(1.0 / rate), rezeroer.on_timer)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
