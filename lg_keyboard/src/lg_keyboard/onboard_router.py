#!/usr/bin/env python
"""
lg_onboard router ROS node implementation.
"""


import rospy
from lg_common.msg import GenericMessage
from lg_common.msg import StringArray
from lg_common import helpers


ROS_NODE_NAME = "lg_onboard_router"


class OnboardRouter:
    """
    - handle initial state
    - have a default viewport for showing onboard
    - listen on director messages
    - idempotently listen on /onboard/visibility
    - publish StringArray with active viewport
    - 
    """

    def __init__(self):
        pass




def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)

    rospy.loginfo("Started lg_onboard_router, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
