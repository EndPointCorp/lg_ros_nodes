#!/usr/bin/env python
"""
lg_onboard router ROS node implementation.
"""


import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_common.msg import StringArray
from lg_common.helpers import route_touch_to_viewports
from lg_common.helpers import load_director_message


ROS_NODE_NAME = "lg_onboard_router"


class DefaultViewportException(Exception):
    pass


class OnboardRouter:
    """
    - handle initial state
    - have a default viewport for showing onboard
    - listen on director messages
    - idempotently listen on /onboard/visibility
    - publish StringArray with viewport that onboard should be shown on /lg_onboard/activate
    - make it provide service with active viewport
    """
    def __init__(
        self,
        default_viewport=[],
        onboard_activate_publisher=None
    ):

        self.onboard_activate_publisher = onboard_activate_publisher
        self.default_viewport = default_viewport
        self.active_viewport = default_viewport

    def handle_scene(self, scene):
        scene = load_director_message(scene)
        windows = scene.get('windows', None)
        if windows:
            received_active_viewport = route_touch_to_viewports(windows, route_touch_key='route_touch', activity_type='mirror')
            if received_active_viewport:
                self.active_viewport = received_active_viewport
            else:
                # no viewports received - falling back to default
                self.active_viewport = self.default_viewport

    def handle_visibility(self, visibility):
        if visibility.data is False:
            self.onboard_activate_publisher.publish(StringArray([]))
        else:
            active_viewport_msg = StringArray(self.active_viewport)
            self.onboard_activate_publisher.publish(active_viewport_msg)
