import subprocess
import threading

import rospy


class ManagedWindow:
    def __init__(self, w_name=None, w_class=None, w_instance=None, x=None, y=None, w=None, h=None, visible=True):
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.is_visible = visible
        #self.proc = None
        self.lock = threading.RLock()

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
        if self.x is not None and self.y is not None:
            cmd.extend([
                'windowmove', self.x, self.y
            ])
        if self.w is not None and self.h is not None:
            cmd.extend([
                'windowsize', self.w, self.h
            ])

    def _visibility_args(self, cmd):
        if self.is_visible:
            cmd.extend(['windowactivate', 'windowfocus'])
        else:
            cmd.append('windowminimize')

    def _sanitize_args(self, cmd):
        return map(str, cmd)

    def set_visibility(self, visible):
        with self.lock:
            self.is_visible = visible
            self.converge()

    def set_geometry(self, x=None, y=None, w=None, h=None):
        with self.lock:
            self.x = x
            self.y = y
            self.w = w
            self.h = h
            self.converge()

    def _get_command(self):
        with self.lock:
            cmd = ['/usr/bin/xdotool']
            self._search_args(cmd)
            self._geometry_args(cmd)
            self._visibility_args(cmd)
        cmd = self._sanitize_args(cmd)
        return cmd

    def converge_once(self):
        with self.lock:
            cmd = self._get_command()
        #if self.proc is not None:
        #    self.proc.stop()
            subprocess.Popen(cmd)

    def converge(self):
        self.converge_once()
        #with self.lock:
        #    cmd = self._get_command()
        #    if self.proc is not None:
        #        self.proc.stop()
        #    self.proc = ProcController(cmd)
        #    self.proc.start()
