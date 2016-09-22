#!/usr/bin/env python
"""
lg_onboard ROS node implementation.

"""


import rospy
from std_msgs.msg import String, Bool

from lg_common import helpers


ROS_NODE_NAME = "lg_onboard"


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)

    #rospy.on_shutdown( ... method ...)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
