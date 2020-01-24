#!/usr/bin/env python3

"""Image Checker

   Cross references the current image_viewer procs with the
   image assets in the last /director/scene msg.  If there are lingering procs
   kill it with fire
"""

import json
import time
import rospy
import subprocess
import threading
from lg_msg_defs.srv import USCSMessage
from interactivespaces_msgs.msg import GenericMessage
from copy import copy

PGREP_CMD = ['pgrep', '-a', 'feh']
EGREP_CMD = ['egrep', '-o', 'http?://[^ ]+']
PARTIAL_KILLER_CMD = ['pkill', '-9', '-f']
IMAGE_PROCS_TO_KILL = ['image_viewer.py', '/usr/bin/feh', '/usr/bin/pqiv']
LOOP_TIMEOUT = 5


class ImageChecker():
    def __init__(self, viewports, timeout_length):
        self.viewports = viewports
        self.timeout_length = timeout_length
        self._timer = None
        self.current_state = None

    def handle_director(self, data):
        if self._timer:
            rospy.logdebug('IMGERROR canceling old timer')
            self._timer.cancel()
        rospy.logdebug('IMGERROR handle_director')
        message = json.loads(data.message)
        self.current_state = message
        self._timer = threading.Timer(self.timeout_length, self.check_image_assets, (message,))
        self._timer.daemon = True
        self._timer.start()

    def check_image_assets(self, message):
        rospy.logdebug('IMGERROR checking image assets msg ')
        feh_image_assets = []
        pqiv_image_assets = []
        for window in message.get('windows', []):
            if window.get('activity', '') == 'image':
                image_viewport = window.get('presentation_viewport', '')
                if image_viewport in self.viewports:
                    if window.get('transparent', False):
                        pqiv_image_assets.append(window['assets'][0])
                    else:
                        feh_image_assets.append(window['assets'][0])

        feh_assets = self._get_feh_assets()
        for feh_asset in feh_assets:
            if feh_asset not in feh_image_assets or len(feh_assets) > len(feh_image_assets):
                time.sleep(0)
                if message != self.current_state:
                    rospy.logdebug('IMGERROR message does not match, leaving this')
                    break
                rospy.logdebug('IMGERROR ASSETS TO REMOVE {}'.format(feh_asset))
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
            feh_assets = feh_proc_assets[0].strip().split(b'\n')
        return feh_assets


def main():
    rospy.init_node('image_checker')
    viewports = [param.strip() for param in rospy.get_param('~viewports', '').split(',')]
    timeout_length = rospy.get_param('~timeout_length', 2)
    checker = ImageChecker(viewports, timeout_length)
    rospy.Subscriber('/director/scene', GenericMessage, checker.handle_director)
    rospy.spin()


if __name__ == '__main__':
    main()
