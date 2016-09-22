#!/usr/bin/env python

import rospy

from lg_common.msg import StringArray

from lg_keyboard import ROS_NODE_NAME
from lg_keyboard import OnboardManager


def main():
    rospy.init_node(ROS_NODE_NAME)
    onboard_mgr = OnboardManager()
    onboard_activate_subscriber = rospy.Subscriber("/lg_onboard/activate",
                                                   StringArray,
                                                   onboard_mgr.handle_activate)
    rospy.on_shutdown(onboard_mgr.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()


if __name__ == "__main__":
    main()
