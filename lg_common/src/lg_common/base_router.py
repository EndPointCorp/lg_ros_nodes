DEFAULT_BASE_ACTIVITIES = {
    'earth': {'earth', 'lg_earth'},
    'cesium': {'cesium', 'lg_cesium'},
    'unreal': {'unreal', 'lg_unreal'},
}


class BaseRouter(object):
    """Remember the base globe selected by explicit Director scenes.

    The configured default is used only at startup. Scenes without a base
    declaration retain the current selection.
    """

    def __init__(self, default_base='earth', activities=None, on_change=None):
        self.activities = activities or DEFAULT_BASE_ACTIVITIES
        self.on_change = on_change
        self.selected = None
        self.select(default_base)

    def select(self, base):
        if base not in self.activities:
            raise ValueError('unknown base application: {}'.format(base))
        if base == self.selected:
            return False
        self.selected = base
        if self.on_change:
            self.on_change(base)
        return True

    def handle_scene(self, scene):
        for window in scene.get('windows', []):
            # Some internal windows deliver an asset to a base application
            # without requesting that application become visible.
            if window.get('select_base', True) is False:
                continue
            activity = window.get('activity')
            for base, activities in self.activities.items():
                if activity in activities:
                    return self.select(base)

        # Overlay and cleanup scenes contain no base. Keep the current one.
        return False
