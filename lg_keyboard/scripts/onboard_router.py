#!/usr/bin/env python

import rospy

from functools import partial
from lg_keyboard.onboard_router import OnboardRouter
from lg_common.helpers import load_director_message
from lg_common.helpers import handle_initial_state
from lg_common.helpers import on_new_scene
from lg_common.msg import StringArray
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import Bool


def main():
    rospy.init_node('lg_keyboard_onboard_router')
    default_viewport = rospy.get_param('~default_viewport', None)
    if not default_viewport:
        raise Exception("Could not get default viewport - please set the rosparam")

    onboard_activate_publisher = rospy.Publisher(
        '/lg_onboard/activate',
        StringArray,
        queue_size=10
    )

    onboard_router = OnboardRouter(
        default_viewport=[default_viewport],
        onboard_activate_publisher=onboard_activate_publisher
    )

    handle_initial_state(onboard_router.handle_scene)
    rospy.Subscriber(
        "/director/scene",
        GenericMessage,
        onboard_router.handle_scene
    )

    rospy.Subscriber(
        "/lg_onboard/visibility",
        Bool,
        onboard_router.handle_visibility
    )

    rospy.loginfo("Started lg_onboard_router, spinning")

    rospy.spin()

if __name__ == "__main__":
    main()
