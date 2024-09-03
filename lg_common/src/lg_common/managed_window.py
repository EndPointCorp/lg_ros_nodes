import json
import rospy
import subprocess
import threading
import re

from lg_msg_defs.msg import WindowGeometry

from lg_common.logger import get_logger
logger = get_logger('managed_window')


class ManagedWindow(object):
    LAYER_BELOW = 'below'
    LAYER_NORMAL = 'normal'
    LAYER_ABOVE = 'above'
    LAYER_TOUCH = 'touch'  # touch things are always on top

    def __init__(self, w_name=None, w_class=None, w_instance=None,
                 geometry=None, visible=True, chrome_kiosk_workaround=False,
                 layer=LAYER_NORMAL):
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance
        self.geometry = geometry
        self.is_visible = visible
        self.layer = layer

        self.lock = threading.Lock()

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
    def format_geometry(geometry):
        """
        Formats WindowGeometry as a string.
        """
        return "{}x{}{:+}{:+}".format(geometry.width, geometry.height,
                                      geometry.x, geometry.y)

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
        if rospy.has_param('~viewport'):
            viewport = rospy.get_param('~viewport')
            geometry = ManagedWindow.lookup_viewport_geometry(viewport)
        else:
            geometry = None

        return geometry

    def _get_command(self):
        msg = {
            'op': 'converge',
            'data': {}
        }

        if self.w_name:
            msg['data']['wm_name'] = self.w_name
        if self.w_instance:
            msg['data']['wm_instance'] = self.w_instance
        if self.w_class:
            msg['data']['wm_class'] = self.w_class

        if self.geometry:
            msg['data']['rectangle'] = ManagedWindow.format_geometry(self.geometry)

        if self.layer:
            msg['data']['layer'] = self.layer

        msg['data']['hidden'] = not self.is_visible

        return ['lg_wm_send', json.dumps(msg, ensure_ascii=False)]

    def set_visibility(self, visible):
        with self.lock:
            self.is_visible = visible

    def set_geometry(self, geometry):
        with self.lock:
            self.geometry = geometry

    def converge(self):
        with self.lock:
            cmd = self._get_command()
            logger.warning('running: {}'.format(cmd))

            try:
                subprocess.check_call(cmd, close_fds=True)
            except Exception as e:
                logger.error('failed to run {} : {}'.format(cmd, str(e)))

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
