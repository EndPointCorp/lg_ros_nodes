import re
import rospy
import threading

from constants import MIRROR_ACTIVITY_TYPE
from constants import MIRROR_TOUCH_CONFIG_KEY
from lg_common.helpers import route_touch_to_viewports


class SubscribeListener:
    def __init__(self, publish_callback):
        self.publish_callback = publish_callback

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.logdebug("New subscription. %s / %s / %s" % (
            topic_name,
            self.publish_callback,
            self.publish_callback)
        )

        self.publish_callback(topic_name)

    def peer_unsubscribe(self, topic_name, num_peers):
        pass


class TouchRouter:
    def __init__(self, default_viewport=None):
        if default_viewport is None:
            self.default_viewports = set()
        else:
            self.default_viewports = set([default_viewport])

        self.route_viewports = self.default_viewports
        self.lock = threading.Lock()

    def handle_service_request(self, req):
        """
        Returns a string[] of active viewports
        """
        with self.lock:
            if len(self.route_viewports) == 0:
                return self.default_viewports
            else:
                return self.route_viewports

    def handle_scene(self, publish_cb, scene):
        """
        Handles an incoming director scene by publishing the list of viewports
        that should have touches routed to them.

        Args:
            publish_cb (function): Callback for publishing the list of
                viewports.
            scene (dict): Director scene.
        """
        with self.lock:
            windows = scene.get('windows', [])
            route_viewports = route_touch_to_viewports(windows, route_touch_key=MIRROR_TOUCH_CONFIG_KEY)
            self.route_viewports = route_viewports

            if len(route_viewports) == 0:
                route_viewports = self.default_viewports

            publish_cb(frozenset(route_viewports))

    def handle_new_listener(self, publish_cb, data):
        """
        Handles new listener connection

        Args:
            publish_cb (function): Callback for publishing the list of
                viewports.
            data: data about new listener
        """
        with self.lock:
            rospy.loginfo("New listener %s" % data)

            if len(self.route_viewports) == 0:
                self.route_viewports = self.default_viewports

            publish_cb(self.route_viewports)
