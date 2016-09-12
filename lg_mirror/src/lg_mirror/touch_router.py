from constants import MIRROR_ACTIVITY_TYPE
from constants import MIRROR_TOUCH_CONFIG_KEY


class TouchRouter:
    def __init__(self, default_viewport):
        if default_viewport is None:
            self.default_viewports = []
        else:
            self.default_viewports = [default_viewport]

    def handle_scene(self, publish_cb, scene):
        windows = scene.get('windows', [])

        def should_route(window):
            activity = w.get('activity')
            config = w.get('activity_config')
            return (
                activity == MIRROR_ACTIVITY_TYPE and
                MIRROR_TOUCH_CONFIG_KEY in config and
                config.get(MIRROR_TOUCH_CONFIG_KEY) is True
            )

        route_viewports = set([
            w.get('presentation_viewport') for w in windows if should_route(w)
        ])

        if len(route_viewports) == 0:
            route_viewports = self.default_viewports

        publish_cb(route_viewports)
