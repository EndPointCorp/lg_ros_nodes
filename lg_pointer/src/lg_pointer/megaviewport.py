import math

from lg_common import ManagedWindow


class MegaViewport:
    def __init__(self, viewports, arc_width):
        if not len(viewports) > 0:
            raise Exception(
                'Must provide at least one viewport for MegaViewport!'
            )
        self.viewports = viewports

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

        self.num_viewports = len(viewports)
        self.overall_width = x_max - x_min
        self.overall_height = y_max - y_min
        aspect_ratio = self.overall_width / self.overall_height

        self.arc_width = arc_width
        self.viewport_width = self.arc_width / self.num_viewports
        self.arc_height = arc_width / aspect_ratio

    def orientation_to_coords(self, ang_z, ang_x):
        half_arc_width = self.arc_width / 2
        half_arc_height = self.arc_height / 2
        nz = ang_z + half_arc_width
        nx = ang_x + half_arc_height
        if 0 > nz or nz > self.arc_width or 0 > nx or nx > self.arc_height:
            return ('', 0, 0)

        viewport_index = int(math.floor(nz / self.viewport_width))
        viewport_x = nz % self.viewport_width / self.viewport_width
        viewport_y = nx / self.arc_height

        viewport = self.viewports[viewport_index]
        return (viewport, viewport_x, viewport_y)
