from lg_common import ManagedWindow
from lg_common.msg import WindowGeometry


def combine_viewport_geometries(viewport_names):
    geometries = [ManagedWindow.lookup_viewport_geometry(v) for v in viewport_names]

    combined = WindowGeometry(
        x=geometries[0].x,
        y=geometries[0].y,
        width=geometries[0].width,
        height=geometries[0].height
    )

    for geometry in geometries:
        combined.x = min(combined.x, geometry.x)
        combined.y = min(combined.y, geometry.y)
        combined.width = max(combined.width, geometry.x - combined.x + geometry.width)
        combined.height = max(combined.height, geometry.y - combined.y + geometry.height)

    return combined
