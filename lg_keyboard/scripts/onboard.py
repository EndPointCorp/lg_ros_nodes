#!/usr/bin/env python

import rospy

from lg_common.msg import StringArray
from lg_keyboard import ROS_NODE_NAME
from lg_keyboard import OnboardLauncher
from lg_keyboard import OnboardViewportException
from lg_common.helpers import run_with_influx_exception_handler


def main():
    rospy.init_node(ROS_NODE_NAME)
    viewport = rospy.get_param("~viewport", None)
    config_path = rospy.get_param("~config_path", None)

    if not viewport:
        message = "No viewport set for OnboardManager - dying"
        rospy.logerr(message)
        raise OnboardViewportException(message)

    onboard_launcher = OnboardLauncher(config_path, viewport)
    onboard_activate_subscriber = rospy.Subscriber("/lg_onboard/activate",
                                                   StringArray,
                                                   onboard_launcher.handle_activate)
    rospy.on_shutdown(onboard_launcher.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(main, ROS_NODE_NAME)
