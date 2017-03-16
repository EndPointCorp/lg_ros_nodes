#!/usr/bin/env python

import rospy
from lg_nav_to_device import DeviceWriter, BackgroundStopper
from lg_common.helpers import run_with_influx_exception_handler
from lg_common.msg import ApplicationState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from interactivespaces_msgs.msg import GenericMessage

NODE_NAME = 'lg_nav_to_device'
DEFAULT_DISABLE_ACTIVITIES = 'cesium,sketchfab,streetview,panovideo'


def main():
    rospy.init_node(NODE_NAME)

    disable_activities = rospy.get_param('~disable_activities', DEFAULT_DISABLE_ACTIVITIES)
    disable_activities = [a.strip() for a in disable_activities.split(',')]
    scale = rospy.get_param('~scale', 512.0)

    device_writer = DeviceWriter(scale)
    rospy.Subscriber('/spacenav_wrapper/twist', Twist, device_writer.make_event)
    rospy.Subscriber('/earth/state', ApplicationState, device_writer.set_state)

    background_stopper = BackgroundStopper(disable_activities, device_writer)
    rospy.Subscriber('/director/scene', GenericMessage, background_stopper.handle_scene)
    rospy.Subscriber('/earth/disable_nav_for_scene_slug', String, background_stopper.handle_slug)

    rospy.spin()

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
