#!/usr/bin/env python
import os
import rospy
import subprocess
from std_msgs.msg import Byte
from lg_common.helpers import run_with_influx_exception_handler


DEVNULL = open(os.devnull, 'w')
CLEAR_BUTTON = 2
RELAUNCH_COMMAND = "pkill -f chrome"
NODE_NAME = 'rfreceiver_relaunch'


class RfreceiverAction:
    def __init__(self):
        rospy.init_node(NODE_NAME)
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
    run_with_influx_exception_handler(RfreceiverAction().main, NODE_NAME)
