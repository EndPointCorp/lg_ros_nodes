#!/usr/bin/env python

import rospy
from functools import partial
from lg_nav_to_device import DeviceWriter, BackgroundStopper
from lg_common.helpers import run_with_influx_exception_handler
from lg_common.msg import ApplicationState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from interactivespaces_msgs.msg import GenericMessage

NODE_NAME = 'lg_nav_to_device'
DEFAULT_DISABLE_ACTIVITIES = 'cesium,unity,sketchfab,streetview,panovideo'
DEFAULT_DISABLE_STATES = '/streetview/state'


def main():
    rospy.init_node(NODE_NAME)

    disable_activities = rospy.get_param('~disable_activities', DEFAULT_DISABLE_ACTIVITIES)
    disable_activities = [a.strip() for a in disable_activities.split(',')]
    disable_states = rospy.get_param('~disable_states', DEFAULT_DISABLE_STATES)
    disable_states = [s.strip() for s in disable_states.split(',')]
    scale = rospy.get_param('~scale', 512.0)

    device_writer = DeviceWriter(scale)
    rospy.Subscriber('/lg_twister/twist', Twist, device_writer.make_event)
    rospy.Subscriber('/earth/state', ApplicationState, device_writer.set_state)

    background_stopper = BackgroundStopper(disable_activities, device_writer)
    rospy.Subscriber('/director/scene', GenericMessage, background_stopper.handle_scene)
    rospy.Subscriber('/earth/disable_nav_for_scene_slug', String, background_stopper.handle_slug)

    for state_topic in disable_states:
        cb = partial(background_stopper.handle_disabled_state, state_topic)
        rospy.Subscriber(state_topic, ApplicationState, cb)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
