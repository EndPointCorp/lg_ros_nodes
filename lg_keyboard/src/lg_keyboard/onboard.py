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
    def __init__(self, viewport=None, launcher=None):
        self.viewport = viewport
        self.launcher = launcher
        if not self.viewport:
            rospy.logerr("No viewport was configured for OnboardManager")
        if not self.launcher:
            rospy.logerr("No launcher was passed to OnboardManager")

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
        if len(message.strings) > 0:
            if self.viewport in message.strings:
                self._show_keyboard()
        else:
            self._hide_keyboard()

    def on_shutdown(self):
        rospy.loginfo("Received shutdown request.")


class OnboardLauncher(object):
    def __init__(self, config_path, viewport=None):
        self.viewport = viewport
        self.selected = False
        self.app = None
        self.window = None
        viewport_geometry = ManagedWindow.lookup_viewport_geometry(self.viewport)
        cmd = ['/usr/bin/onboard', '-m']

        if self.viewport is not None:
            onboard_width = viewport_geometry.width
            onboard_height = viewport_geometry.height / 4

            onboard_geometry = WindowGeometry(
                x=viewport_geometry.x,
                y=viewport_geometry.height,
                width=viewport_geometry.width,
                height=viewport_geometry.height
            )

            self.config = OnboardConfig(
                config_path=config_path,
                geometry=onboard_geometry
                ).get_config()

            self.window = ManagedWindow(
                w_name='onboard',
                geometry=onboard_geometry,
                visible=False
            )

            x = onboard_geometry.x
            y = viewport_geometry.height - onboard_height
            cmd.extend(map(str, [
                '-x', x,
                '-y', y,
                '-s', '{}x{}'.format(viewport_geometry.width, onboard_height)
            ]))
            self.cmd = cmd
        else:
            message = "Could not find viewport for OnboardLauncher - dying"
            rospy.logerr(message)
            raise OnboardViewportException(message)


    def show_onboard(self):
        """
        Idempotently shows onboard window
        """
        # onboard -m -x -y -s

        self.app = ManagedApplication(self.cmd, self.window)
        dconf = subprocess.Popen(['/usr/bin/dconf', 'load', '/org/onboard/'],
                                 stdin=subprocess.PIPE,
                                 close_fds=True)
        rospy.loginfo("Using config => %s" % self.config)
        dconf.communicate(input=self.config)

        self.app.set_state(ApplicationState.VISIBLE)

    def hide_onboard(self):
        """
        Idemmpotently hides onboard window
        """
        if self.app:
            self.app.set_state(ApplicationState.STOPPED)


class OnboardConfig(object):
    def __init__(self, config_path=None, geometry=None):
        self.geometry = geometry
        self.config_path = config_path
        config_path = rospkg.RosPack().get_path('lg_keyboard') + "/config"
        self.config = self._read_onboard_config(config_path)

    def _read_onboard_config(self, config_path):
        default_config_file = config_path + "/onboard-default.dconf"

        with open(default_config_file) as cf:
            config = cf.read().replace('{config_path}', config_path)
            if self.geometry is not None:
                config = config.replace('{docking}', 'false')
            else:
                config = config.replace('{docking}', 'true')

        return config

    def get_config(self):
        rospy.loginfo("Returning onboard config: %s" % self.config)
        return self.config
