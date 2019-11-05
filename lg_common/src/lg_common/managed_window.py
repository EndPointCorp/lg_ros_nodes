import rospy
import subprocess
import threading
import re

from lg_common.msg import WindowGeometry
from .devilspie2 import Devilspie2


class ManagedWindow(object):
    def __init__(self, w_name=None, w_class=None, w_instance=None,
                 geometry=None, visible=True, chrome_kiosk_workaround=False):
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance
        self.geometry = geometry
        self.is_visible = visible

        self._lock = threading.RLock()
        self._manager = None

        rospy.on_shutdown(self._cleanup_manager)

    def __str__(self):
        return 'name={name}, class={cls}, instance={inst}, {w}x{h} {x},{y}'.format(
            name=self.w_name,
            cls=self.w_class,
            inst=self.w_instance,
            w=self.geometry.width if self.geometry is not None else None,
            h=self.geometry.height if self.geometry is not None else None,
            x=self.geometry.x if self.geometry is not None else None,
            y=self.geometry.y if self.geometry is not None else None,
        )

    @staticmethod
    def parse_geometry(geometry):
        """
        Parses Xorg window geometry in the form WxH[+-]X[+-]Y

        Raises ValueError if the geometry string is invalid.
        """
        m = re.match(r'^(\d+)x(\d+)([+-]\d+)([+-]\d+)$', geometry)

        if m is None:
            raise ValueError(
                'Invalid window geometry: {}'.format(geometry))

        dims = list(map(int, m.groups()))
        return WindowGeometry(width=dims[0], height=dims[1],
                              x=dims[2], y=dims[3])

    @staticmethod
    def lookup_viewport_geometry(viewport_key):
        """
        Looks up geometry for the given viewport name.

        Raises KeyError if the viewport is not configured.
        """
        param_name = '/viewport/{}'.format(viewport_key)

        if not rospy.has_param(param_name):
            raise KeyError(
                'Viewport parameter not set: {}'.format(param_name))

        viewport_value = rospy.get_param(param_name)
        return ManagedWindow.parse_geometry(viewport_value)

    @staticmethod
    def get_viewport_geometry():
        """
        Returns WindowGeometry if the private '~viewport' param is set.

        Returns None if the private '~viewport' param is not set.
        """
        viewport = rospy.get_param('~viewport', None)
        if viewport:
            return ManagedWindow.lookup_viewport_geometry(viewport)
        else:
            return None

    def set_visibility(self, visible):
        with self._lock:
            self.is_visible = visible

    def set_geometry(self, geometry):
        with self._lock:
            self.geometry = geometry

    def _cleanup_manager(self):
        if self._manager:
            self._manager.close()
            self._manager = None

    def converge(self):
        with self._lock:
            self._cleanup_manager()
            self._manager = Devilspie2(
                self.w_name,
                self.w_class,
                self.w_instance,
                self.geometry,
                self.is_visible,
            )

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
