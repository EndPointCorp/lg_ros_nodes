"""
lg_onboard ROS node implementation.

"""


import rospy
import subprocess

from lg_common import ManagedApplication
from lg_common import ManagedWindow
from lg_msg_defs.msg import ApplicationState
from lg_msg_defs.msg import WindowGeometry


ROS_NODE_NAME = "lg_onboard"
from lg_common.helpers import get_package_path
from lg_common.logger import get_logger
logger = get_logger(ROS_NODE_NAME)


class OnboardViewportException(Exception):
    pass


class OnboardLauncher(object):

    def __init__(self, config_path, viewport):
        self.config_path = config_path
        self.viewport = viewport
        self.selected = False

        viewport_geometry = ManagedWindow.lookup_viewport_geometry(self.viewport)
        cmd = ['/usr/bin/onboard', '-m']

        onboard_width = int(viewport_geometry.width)
        onboard_height = int(viewport_geometry.height / 6)
        onboard_x = int(viewport_geometry.x)
        onboard_y = int(viewport_geometry.y + (viewport_geometry.height - onboard_height))

        onboard_geometry = WindowGeometry(x=onboard_x,
                                          y=onboard_y,
                                          width=onboard_width,
                                          height=onboard_height)

        self.config = OnboardConfig(config_path=config_path).get_config()

        window = ManagedWindow(w_class='Onboard',
                               geometry=onboard_geometry,
                               layer=ManagedWindow.LAYER_TOUCH,
                               visible=False)
        cmd.extend(list(map(str, [
            '-x',
            onboard_x,
            '-y',
            onboard_y,
            '-s',
            '{}x{}'.format(onboard_width, onboard_height)
        ])))
        self.app = ManagedApplication(cmd, window=window)

    def handle_activate(self, message):
        """
        Call show_keyboard to idempotently show kayboard
        if is self.viewport was emitted on the list of
        viewports in the incoming message.

        Call hide_keyboard to idempotently close keyboard
        if self.viewport was not emitted on the incoming list

        """
        if self.viewport in message.strings:
            self.show_onboard()
        else:
            self.hide_onboard()

    def show_onboard(self):
        """
        Idempotently shows onboard window.

        """
        logger.info("Using config => %s" % self.config)
        dconf = subprocess.Popen(['/usr/bin/dconf', 'load', '/org/onboard/'],
                                 stdin=subprocess.PIPE,
                                 close_fds=True)
        dconf.communicate(input=self.config.encode())
        self.app.set_state(ApplicationState.VISIBLE)

    def hide_onboard(self):
        """
        Idempotently hides onboard window.

        """
        self.app.set_state(ApplicationState.STOPPED)

    def on_shutdown(self):
        logger.info("Received shutdown request.")


class OnboardConfig(object):
    def __init__(self, config_path=None):
        self.config_path = config_path
        config_path = get_package_path('lg_keyboard') + "/config"
        self.config = self._read_onboard_config(config_path)

    def _read_onboard_config(self, config_path):
        default_config_file = config_path + "/onboard-default.dconf"
        with open(default_config_file, 'rb') as cf:
            config = cf.read().decode('utf-8')
        config = config.replace('{config_path}', config_path)
        return config

    def get_config(self):
        logger.info("Returning onboard config: %s" % self.config)
        return self.config
