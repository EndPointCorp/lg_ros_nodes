import re

from constants import MIRROR_ACTIVITY_TYPE
from constants import MIRROR_TOUCH_CONFIG_KEY


class TouchRouter:
    def __init__(self, default_viewport=None):
        if default_viewport is None:
            self.default_viewports = set()
        else:
            self.default_viewports = set([default_viewport])

    @staticmethod
    def maybe_route(routed, window):
        """
        Reduces all windows in the scene to the set of viewports that should
        be receiving touch events.

        Args:
            routed (set): Accumulated set of viewports that should be
                receiving touch events.
            window (dict): Window from a director scene.

        Returns:
            set(str): Accumulated set of viewports that should be receiving
                touch events.
        """
        activity = window.get('activity')
        if activity != MIRROR_ACTIVITY_TYPE:
            return routed

        config = window.get('activity_config', {})
        if MIRROR_TOUCH_CONFIG_KEY not in config:
            return routed
        if config.get(MIRROR_TOUCH_CONFIG_KEY) is not True:
            return routed

        source = config.get('viewport', '')
        viewport = re.sub(r'^viewport:\/\/', '', source, count=1)
        routed.add(viewport)
        return routed

    def handle_scene(self, publish_cb, scene):
        """
        Handles an incoming director scene by publishing the list of viewports
        that should have touches routed to them.

        Args:
            publish_cb (function): Callback for publishing the list of
                viewports.
            scene (dict): Director scene.
        """
        windows = scene.get('windows', [])

        route_viewports = reduce(TouchRouter.maybe_route, windows, set())

        if len(route_viewports) == 0:
            route_viewports = self.default_viewports

        publish_cb(frozenset(route_viewports))
