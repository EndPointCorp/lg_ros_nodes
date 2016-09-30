#!/usr/bin/env python
"""
lg_onboard router ROS node implementation.

"""


import rospy
import threading

from lg_common.msg import StringArray
from lg_common.helpers import route_touch_to_viewports
from lg_common.helpers import load_director_message


class OnboardRouter:
    """
    - handle initial state
    - have a default viewport for showing onboard
    - listen on director messages
    - idempotently listen on /onboard/visibility
    - publish StringArray with viewport that onboard should be shown on /lg_onboard/activate
    - make it provide service with active viewport

    """
    def __init__(self, default_viewport=[], onboard_activate_publisher=None):
        self.lock = threading.Lock()
        self.last_state = None
        self.onboard_activate_publisher = onboard_activate_publisher
        self.default_viewport = default_viewport
        self.active_viewport = default_viewport

    def handle_scene(self, scene):
        with self.lock:
            scene = load_director_message(scene)
            windows = scene.get('windows', None)
            if windows:
                self._hide_onboard()
                received_active_viewport = route_touch_to_viewports(
                    windows,
                    route_touch_key='route_touch',
                    activity_type='mirror')
                if received_active_viewport:
                    self.active_viewport = received_active_viewport
                else:
                    # no viewports received - falling back to default
                    self.active_viewport = self.default_viewport
            else:
                self._hide_onboard()

    def _hide_onboard(self):
        """
        Publishes empty string of viewports to
        hide onboard on all of them

        """
        rospy.loginfo("HIDING onboard")
        self.onboard_activate_publisher.publish(StringArray([]))

    def handle_visibility(self, visibility):
        with self.lock:
            if visibility.data is False:
                if not (self.last_state == visibility.data):
                    rospy.loginfo("HIDING onboard")
                    self._hide_onboard()
                    self.last_state = False
            else:
                if not (self.last_state == visibility.data):
                    rospy.loginfo("SHOWING onboard on %s" % self.active_viewport)
                    active_viewport_msg = StringArray(self.active_viewport)
                    self.onboard_activate_publisher.publish(active_viewport_msg)
                    self.last_state = True
