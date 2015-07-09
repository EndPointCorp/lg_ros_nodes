import rospy
import subprocess
import threading
import re

from lg_common.msg import WindowGeometry

XDOTOOL_BIN = '/usr/bin/xdotool'


class ManagedWindow(object):
    def __init__(self, w_name=None, w_class=None, w_instance=None,
                 geometry=None, visible=True):
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance
        self.geometry = geometry
        self.is_visible = visible
        self.lock = threading.RLock()
        self.proc = None

        rospy.on_shutdown(self._cleanup_proc)

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

        dims = map(int, m.groups())
        return WindowGeometry(width=dims[0], height=dims[1],
                              x=dims[2], y=dims[3])

    @staticmethod
    def lookup_viewport(viewport_key):
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
        if rospy.has_param('~viewport'):
            viewport = rospy.get_param('~viewport')
            geometry = ManagedWindow.lookup_viewport(viewport)
        else:
            geometry = None

        return geometry

    def _search_args(self, cmd):
        if self.w_class is not None:
            cmd.extend([
                'search', '--sync', '--class', self.w_class
            ])
        if self.w_name is not None:
            cmd.extend([
                'search', '--sync', '--name', self.w_name
            ])
        if self.w_instance is not None:
            cmd.extend([
                'search', '--sync', '--classname', self.w_instance
            ])

    def _geometry_args(self, cmd):
        if self.geometry is not None:
            cmd.extend([
                'windowmove', self.geometry.x, self.geometry.y
            ])
            cmd.extend([
                'windowsize', self.geometry.width, self.geometry.height
            ])

    def _visibility_args(self, cmd):

        if self.is_visible:
            cmd.append('windowactivate')
        else:
            cmd.append('windowminimize')

    def _sanitize_args(self, cmd):
        return map(str, cmd)

    def _get_command(self):
        with self.lock:
            cmd = [XDOTOOL_BIN]
            self._search_args(cmd)
            self._geometry_args(cmd)
            self._visibility_args(cmd)
        cmd = self._sanitize_args(cmd)
        return cmd

    def _cleanup_proc(self):
        with self.lock:
            if self.proc is not None:
                self.proc.kill()

    def set_visibility(self, visible):
        with self.lock:
            self.is_visible = visible

    def set_geometry(self, geometry):
        with self.lock:
            self.geometry = geometry

    def converge(self):
        with self.lock:
            cmd = self._get_command()
            self._cleanup_proc()
            rospy.loginfo(cmd)
            try:
                self.proc = subprocess.Popen(cmd)
            except OSError:
                rospy.logerror('failed to run {}'.format(XDOTOOL_BIN))

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
