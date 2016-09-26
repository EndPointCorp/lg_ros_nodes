#!/usr/bin/env python

import rospy

from lg_common.msg import StringArray

from lg_keyboard import ROS_NODE_NAME
from lg_keyboard import OnboardManager
from lg_keyboard import OnboardLauncher
from lg_keyboard import OnboardViewportException


def main():
    rospy.init_node(ROS_NODE_NAME)
    viewport = rospy.get_param("~viewport", None)
    config = rospy.get_param("~config", None)

    if not viewport:
        message = "No viewport set for OnboardManager - dying"
        rospy.logerror(message)
        raise OnboardViewportException(message)

    onboard_launcher = OnboardLauncher(
        viewport=viewport,
        config=config
    )
    onboard_mgr = OnboardManager(
        viewport=viewport,
        launcher=onboard_launcher
    )

    onboard_activate_subscriber = rospy.Subscriber("/lg_onboard/activate",
                                                   StringArray,
                                                   onboard_mgr.handle_activate)
    rospy.on_shutdown(onboard_mgr.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()


if __name__ == "__main__":
    main()
