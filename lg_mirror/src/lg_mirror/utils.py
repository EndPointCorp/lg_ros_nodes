import rospy
from constants import MULTICAST_GROUP


def viewport_to_multicast_group(viewport_key):
    """
    Looks up the multicast group for the given viewport.

    This is done by getting all the viewports, sorting their keys, and
    looking up the index of the given viewport name.

    Args:
        viewport_key (str)

    Returns:
        addr (str): Multicast group for the given viewport.

    Raises:
        KeyError: The viewport is not configured.
    """
    viewports = rospy.get_param('/viewport')

    if viewports is None:
        raise KeyError("No viewports configured")

    if viewport_key not in viewports.keys():
        raise KeyError("Viewport {} is not configured".format(viewport_key))

    sorted_viewport_keys = sorted(viewports.keys())
    i = sorted_viewport_keys.index(viewport_key)

    addr = MULTICAST_GROUP.format(i + 1)
    return addr


def get_mirror_port():
    """
    Returns the port that should be used for mirroring RTP.

    Returns:
        int
    """
    return 4953


def aspect_scale_source(source_geometry, dest_geometry):
    """
    Find geometry that fits the destination while preserving the aspect ratio
    of the source.

    This will only ever result in scaling the geometry down, since we generally
    want to reduce the size of the encoded stream and upscaling can be done
    effectively on the receiving end.

    Args:
        source_geometry (WindowGeometry)
        dest_geometry (WindowGeometry)

    Returns:
        int, int: Rounded width and height fitting the destination window.
    """
    source_ratio = float(source_geometry.width) / float(source_geometry.height)

    # Start by matching width.
    width = min(source_geometry.width, dest_geometry.width)
    height = width / source_ratio

    # If height is out of source bounds, start by matching height instead.
    if height > source_geometry.height:
        height = source_geometry.height
        width = height * source_ratio

    # If width is out of destination bounds, match height.
    if height > dest_geometry.height:
        height = dest_geometry.height
        width = height * source_ratio

    return int(round(width)), int(round(height))

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
