import os
import stat
import subprocess


def chmod_x(path):
    st = os.stat(path)
    os.chmod(path, st.st_mode | stat.S_IEXEC)


class Toggle3d:
    def __init__(self):
        self.layer_is_on = True

        # catkin will istall bash scripts as executable without .bash extension
        self.script = '/home/lg/python_scripts/toggle_layer.py'
        chmod_x(self.script)

    def set_layer_state(self, state):
        self.sync_state_to_earth(state)
        self.layer_is_on = state

        if self.state_listener:
            self.state_listener(self.layer_is_on)

    def set_on_change_listener(self, state_listener):
        self.state_listener = state_listener

    def sync_state_to_earth(self, state):
        subprocess.call([self.script, '0' if state else '1'])
