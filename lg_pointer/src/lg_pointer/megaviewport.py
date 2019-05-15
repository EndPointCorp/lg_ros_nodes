import math

from lg_common import ManagedWindow


class MegaViewport:
    """
    Helper for converting angular offsets to viewport coordinates across
    the span of all viewports.

    It is assumed that:
    * All given viewports have the same dimensions.
    * The pointing device is located roughly at the center of the cylinder.
    """

    def __init__(self, viewports, arc_width):
        """
        Args:
            viewports (list[str]): List of viewports from left to right.
            arc_width (float): Physical arc width of all viewports in radians.

        Raises:
            Exception: No viewports provided.
        """
        if not len(viewports) > 0:
            raise Exception(
                'Must provide at least one viewport for MegaViewport!'
            )
        self.viewports = viewports
        self.arc_width = arc_width

        # Calculate aspect ratio of all viewports.
        # Here we do not assume that all viewports have the same dimensions.
        x_min = 16384
        y_min = 16384
        x_max = 0
        y_max = 0
        for viewport in viewports:
            geometry = ManagedWindow.lookup_viewport_geometry(viewport)
            x_min = min(geometry.x, x_min)
            y_min = min(geometry.y, y_min)
            x_max = max(geometry.x + geometry.width, x_max)
            y_max = max(geometry.y + geometry.height, y_max)
        overall_width = x_max - x_min
        overall_height = y_max - y_min
        aspect_ratio = overall_width / overall_height

        num_viewports = len(viewports)
        # It is assumed here that all viewports have the same width.
        self.viewport_width = self.arc_width / num_viewports
        self.arc_height = self.arc_width / aspect_ratio
        self.half_arc_width = self.arc_width / 2
        self.half_arc_height = self.arc_height / 2

    def orientation_to_coords(self, ang_z, ang_x):
        """
        Convert pointer angle to coordinates in a viewport.

        This method assumes that the pointer location is located exactly at
        the center point of the cylinder.  Far from exact, but usually this
        is the only data available.

        Args:
            ang_z (float): Z (heading) rotation in radians.
            ang_x (float): X (pitch) rotation in radians.

        Returns:
            str, int, int: Tuple with viewport name and x/y coordinates.
                If the coordinate is outside all viewports, returns (None, 0, 0)
        """
        if self.clamp(ang_z, ang_x):
            return (None, 0, 0)
        nz = ang_z + self.half_arc_width
        nx = ang_x + self.half_arc_height
        viewport_index = int(math.floor(nz / self.viewport_width))
        viewport_x = nz % self.viewport_width / self.viewport_width
        viewport_y = nx / self.arc_height

        viewport = self.viewports[viewport_index]
        return (viewport, viewport_x, viewport_y)

    def clamp(self, ang_z, ang_x):
        """
        Clamp cursors to the viewable boundary.

        Args:
            ang_z (float): Z (heading) rotation in radians.
            ang_x (float): X (pitch) rotation in radians.

        Returns:
            bool: True if the mouse is still viewable
        """
        nz = ang_z + self.half_arc_width
        nx = ang_x + self.half_arc_height
        if 0 > nz or nz > self.arc_width or 0 > nx or nx > self.arc_height:
            return True
        return False
