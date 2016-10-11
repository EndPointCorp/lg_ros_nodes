import rospy


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


def get_viewport_base_topic(viewport_name):
    """
    Return a good base topic name for the viewport.

    Actual image transport topic names will be based on this name.

    See http://wiki.ros.org/image_transport

    Args:
        viewport_name (str)

    Returns:
        str: Base topic name for the viewport.
    """
    return '/lg_mirror/viewport/{}'.format(viewport_name)


def get_viewport_image_topic(viewport_name):
    """
    Return compressed image topic for a viewport.

    Args:
        viewport_name (str)

    Returns:
        str: Compressed image topic for the viewport.
    """
    return get_viewport_base_topic(viewport_name) + '/compressed'

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
