"""
lg_onboard ROS node implementation.

"""


import rospy
from std_msgs.msg import String, Bool

from lg_common import helpers
from lg_common import ManagedApplication
from lg_common import ManagedWindow
from lg_common.msg import ApplicationState

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
    def __init__(self, config, viewport=None):
        self.viewport = viewport
        self.selected = False
        self.app = None
        self.window = None
        self.config = config

        cmd = ['/usr/bin/onboard', '-m']

        if self.viewport is not None:
            window = ManagedWindow()
            geometry = ManagedWindow.lookup_viewport_geometry(self.viewport)

            width = geometry.width
            height = geometry.height / 4
            x = geometry.x
            y = geometry.y + geometry.height - height
            cmd.extend(map(str, [
                '-x', x,
                '-y', y,
                '-s', '{}x{}'.format(width, height)
            ]))
        else:
            message = "Could not find viewport for OnboardLauncher - dying"
            rospy.logerr(message)
            raise OnboardViewportException(message)

        self.app = ManagedApplication(cmd, window)

    def show_onboard(self):
        """
        Idempotently shows onboard window
        """
        dconf = Popen(['/usr/bin/dconf', 'load', '/org/onboard/'], stdin=PIPE)
        dconf.communicate(input=self.config)

        self.app.set_state(ApplicationState.VISIBLE)

    def hide_onboard(self):
        """
        Idemmpotently hides onboard window
        """
        self.app.set_state(ApplicationState.HIDDEN)
