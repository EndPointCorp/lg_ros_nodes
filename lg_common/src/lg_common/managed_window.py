import rospy
import subprocess
import threading

from lg_common.msg import WindowGeometry

XDOTOOL_BIN = '/usr/bin/xdotool'


class ManagedWindow:
    def __init__(self, w_name=None, w_class=None, w_instance=None, geometry=None, visible=True):
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance
        self.geometry = geometry
        self.is_visible = visible
        self.lock = threading.RLock()
        self.proc = None

        rospy.on_shutdown(self._cleanup_proc)

    def _search_args(self, cmd):
        cmd.extend([
            'search', '--maxdepth', '1', '--limit', '1', '--sync'
        ])

        if self.w_instance is not None:
            cmd.extend([
                '--classname', self.w_instance
            ])
        elif self.w_name is not None:
            cmd.extend([
                '--name', self.w_name
            ])
        elif self.w_class is not None:
            cmd.extend([
                '--class', self.w_class
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
            try:
                self.proc = subprocess.Popen(cmd)
            except OSError:
                rospy.logerror('failed to run {}',format(XDOTOOL_BIN)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
