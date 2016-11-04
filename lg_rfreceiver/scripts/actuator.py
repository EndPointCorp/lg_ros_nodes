#!/usr/bin/env python
import os
import rospy
import subprocess
from std_msgs.msg import Byte


DEVNULL = open(os.devnull, 'w')
CLEAR_BUTTON = 2
RELAUNCH_COMMAND = "pkill -f chrome"


class RfreceiverAction:
    def __init__(self):
        rospy.init_node('rfreceiver_relaunch')
        self.clear_button_message = rospy.get_param('~clear_button_message', 2)

    def handle_button_msg(self, msg):
        if msg.data == self.clear_button_message:
            subprocess.call(
                RELAUNCH_COMMAND.split(' '),
                stdout=DEVNULL,
                stderr=DEVNULL
            )

    def main(self):
        rospy.Subscriber(
            '/rfreceiver/buttondown',
            Byte,
            self.handle_button_msg
        )

        rospy.spin()

if __name__ == '__main__':
    RfreceiverAction().main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
