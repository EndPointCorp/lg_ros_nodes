#!/usr/bin/env python

"""Image Checker

   Cross references the current image_viewer procs with the
   image assets in the last /director/scene msg.  If there are lingering procs
   kill it with fire
"""

import json
import rospy
import subprocess
from lg_common.srv import USCSMessage
from interactivespaces_msgs.msg import GenericMessage
from copy import copy

PGREP_CMD = ['pgrep', '-a', 'feh']
EGREP_CMD = ['egrep', '-o', 'http?://[^ ]+']
PARTIAL_KILLER_CMD = ['pkill', '-9', '-f']
IMAGE_PROCS_TO_KILL = ['image_viewer.py', '/usr/bin/feh', '/usr/bin/pqiv']
LOOP_TIMEOUT = 5


class ImageChecker():
    def __init__(self, last_uscs_message):
        self.last_uscs_service = last_uscs_service

    def handle_director(self, data):
        rospy.logerr('handle_director')
        image_assets = []
        message = json.loads(data.message)
        rospy.sleep(2)
        for window in message.get('windows', []):
            if window.get('activity', '') == 'image':
                image_assets.append(window['assets'][0])

        feh_assets = self._get_feh_assets()
        for feh_asset in feh_assets:
            if feh_asset not in image_assets:
                rospy.logerr('ASSETS TO REMOVE {}'.format(feh_asset))
                if json.loads(self.last_uscs_service().message) == message:
                    for image_proc in IMAGE_PROCS_TO_KILL:
                        subprocess.call(PARTIAL_KILLER_CMD + [image_proc])
                break

    def _get_feh_assets(self):
        feh_assets = []
        pgrep = subprocess.Popen(PGREP_CMD, stdout=subprocess.PIPE)
        feh_proc = subprocess.Popen(
            EGREP_CMD,
            stdin=pgrep.stdout,
            stdout=subprocess.PIPE
        )
        feh_proc_assets = feh_proc.communicate()
        if feh_proc_assets[0]:
            feh_assets = feh_proc_assets[0].strip().split('\n')
        return feh_assets


def main():
    rospy.init_node('image_checker')
    rospy.wait_for_service('/uscs/message', 10)
    last_uscs_service = rospy.ServiceProxy('/uscs/message', USCSMessage)
    checker = ImageChecker(last_uscs_service)
    rospy.Subscriber('/director/scene', GenericMessage, checker.handle_director)
    rospy.spin()

if __name__ == '__main__':
    main()
