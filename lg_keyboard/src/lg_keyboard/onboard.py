"""
lg_onboard ROS node implementation.

"""


import rospy
import rospkg
import subprocess

from std_msgs.msg import String, Bool
from lg_common import helpers
from lg_common import ManagedApplication
from lg_common import ManagedWindow
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry

ROS_NODE_NAME = "lg_onboard"


class OnboardViewportException(Exception):
    pass


class OnboardManager(object):
    """
    Onboard manager handles starting, showing and terminating
    the onboard system application.

    """
    def __init__(self, viewport, launcher):
        self.viewport = viewport
        self.launcher = launcher

    def _show_keyboard(self):
        self.launcher.show_onboard()

    def _hide_keyboard(self):
        self.launcher.hide_onboard()

    def handle_activate(self, message):
        """
        Call show_keyboard to idempotently show kayboard
        if is self.viewport was emitted on the list of
        viewports in the incoming message.

        Call hide_keyboard to idempotently close keyboard
        if self.viewport was not emitted on the incoming list
        """
        if self.viewport in message.strings:
            self._show_keyboard()
        else:
            self._hide_keyboard()

    def on_shutdown(self):
        rospy.loginfo("Received shutdown request.")


class OnboardLauncher(object):
    def __init__(self, config_path, viewport):
        self.viewport = viewport
        self.selected = False

        viewport_geometry = ManagedWindow.lookup_viewport_geometry(self.viewport)
        cmd = ['/usr/bin/onboard', '-m']

        onboard_width = viewport_geometry.width
        onboard_height = viewport_geometry.height / 4
        onboard_x = viewport_geometry.x
        onboard_y = viewport_geometry.y + (viewport_geometry.height - onboard_height)

        onboard_geometry = WindowGeometry(
            x=onboard_x,
            y=onboard_y,
            width=onboard_width,
            height=onboard_height
        )

        self.config = OnboardConfig(
            config_path=config_path
        ).get_config()

        window = ManagedWindow(
            w_class='Onboard',
            geometry=onboard_geometry,
            visible=False
        )

        cmd.extend(map(str, [
            '-x', onboard_x,
            '-y', onboard_y,
            '-s', '{}x{}'.format(onboard_width, onboard_height)
        ]))
        self.app = ManagedApplication(cmd, window=window)

    def show_onboard(self):
        """
        Idempotently shows onboard window
        """
        rospy.loginfo("Using config => %s" % self.config)
        dconf = subprocess.Popen(['/usr/bin/dconf', 'load', '/org/onboard/'],
                                 stdin=subprocess.PIPE,
                                 close_fds=True)
        dconf.communicate(input=self.config)

        self.app.set_state(ApplicationState.VISIBLE)

    def hide_onboard(self):
        """
        Idempotently hides onboard window
        """
        self.app.set_state(ApplicationState.STOPPED)


class OnboardConfig(object):
    def __init__(self, config_path=None):
        self.config_path = config_path
        config_path = rospkg.RosPack().get_path('lg_keyboard') + "/config"
        self.config = self._read_onboard_config(config_path)

    def _read_onboard_config(self, config_path):
        default_config_file = config_path + "/onboard-default.dconf"

        with open(default_config_file) as cf:
            config = cf.read()
        config = config.replace('{config_path}', config_path)

        return config

    def get_config(self):
        rospy.loginfo("Returning onboard config: %s" % self.config)
        return self.config
