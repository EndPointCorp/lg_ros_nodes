#!/usr/bin/env python3
"""
lg_onboard router ROS node implementation.

"""


import rospy
import threading

from lg_msg_defs.msg import StringArray
from lg_common.helpers import route_touch_to_viewports
from lg_common.helpers import load_director_message


class OnboardRouter(object):
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

    def _hide_onboard(self):
        """
        Publishes empty string of viewports to hide onboard on all of them.

        """
        rospy.loginfo("Hiding Onboard ...")
        self.onboard_activate_publisher.publish(StringArray([]))

    def _show_onboard(self):
        """
        Publishes an array of strings with viewports to show onboard on.

        """
        rospy.loginfo("Showing Onboard on %s" % self.active_viewport)
        active_viewport_msg = StringArray(self.active_viewport)
        self.onboard_activate_publisher.publish(active_viewport_msg)

    def handle_scene(self, scene):
        with self.lock:
            scene = load_director_message(scene)
            windows = scene.get('windows', None)
            self._hide_onboard()
            if windows:
                received_active_viewport = route_touch_to_viewports(
                    windows,
                    route_touch_key='route_touch')
                if received_active_viewport:
                    self.active_viewport = list(received_active_viewport)
                else:
                    # no viewports received - falling back to default
                    self.active_viewport = self.default_viewport

    def handle_visibility(self, visibility):
        """
        Process visibility boolean messages.
        Repeated messages of the same value are not processed.
        Only the value flip situation is considered.

        """
        with self.lock:
            if visibility.data == self.last_state:
                return
            if visibility.data is False:
                self._hide_onboard()
                self.last_state = False
            else:
                self._show_onboard()
                self.last_state = True
