"""
lg_onboard ROS node implementation.

"""


import rospy
from std_msgs.msg import String, Bool

from lg_common import helpers


ROS_NODE_NAME = "lg_onboard"


class OnboardManager(object):
    """
    Onboard manager handles starting, showing and terminating
    the onboard system application.

    """
    def __init__(self):
        # list of lg_common.ManagedApplication
        self.onboard_procs = []
        # TODO
        # init dbus as necessary
        pass

    def handle_activate(self, viewports):
        # TODO
        # 1) start onboards on target viewports
        # 2) kill running onboards processes - upon reception of empty array StringArray([])
        pass

    def on_shutdown(self):
        rospy.loginfo("Received shutdown request.")
