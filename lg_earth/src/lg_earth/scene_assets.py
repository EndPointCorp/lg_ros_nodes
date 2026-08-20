try:
    from urllib.parse import urlsplit
except ImportError:  # pragma: no cover - Python 2 compatibility
    from urlparse import urlsplit


GLOBE_ACTIVITIES = {
    'earth': {'earth', 'lg_earth'},
    'cesium': {'cesium', 'lg_cesium'},
    'unreal': {'unreal', 'lg_unreal'},
}


def is_kml_asset(asset):
    """Return whether an asset URL can be shared by globe renderers."""
    if not isinstance(asset, str):
        return False
    path = urlsplit(asset).path.lower()
    return path.endswith('.kml') or path.endswith('.kmz')


def is_base_only_scene(scene):
    """Recognize compatibility scenes that select a base without content."""
    if scene.get('slug') in ('open-earth', 'open-cesium', 'open-unreal'):
        return True
    return any(
        window.get('activity_config', {}).get('force_touchscreen_tab')
        for window in scene.get('windows', [])
    )


def assets_for_renderer(scene, viewport, renderer):
    """Collect native assets plus shareable KML/KMZ for one renderer.

    Native windows retain every asset type understood by that renderer.
    Windows for other globe renderers contribute only KML/KMZ assets.
    """
    native = GLOBE_ACTIVITIES[renderer]
    globe = set().union(*GLOBE_ACTIVITIES.values())
    result = []

    for window in scene.get('windows', []):
        if window.get('presentation_viewport') != viewport:
            continue
        activity = window.get('activity')
        if activity not in globe:
            continue
        assets = window.get('assets', [])
        if activity not in native:
            assets = [asset for asset in assets if is_kml_asset(asset)]
        for asset in assets:
            if asset not in result:
                result.append(asset)

    return result
