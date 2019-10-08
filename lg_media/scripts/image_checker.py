#!/usr/bin/env python
import json
import rospy
import subprocess
from lg_common.srv import USCSMessage
from interactivespaces_msgs.msg import GenericMessage
from copy import copy

PGREP_CMD = ['pgrep', '-a', 'feh']
EGREP_CMD = ['egrep', '-o', 'http?://[^ ]+']
PARTIAL_KILLER_CMD = ['pkill', '-9', '-f']
IMAGE_PROCS_TO_KILL = ['image_viewer.py', 'feh', 'pqiv']
LOOP_TIMEOUT = 5


class ImageChecker():
    def __init__(self):
        self.last_message = ''

    def handle_director(self, data):
        assets_to_remove = []
        new_image_windows = []
        message = json.loads(data.message)
        self.last_message = copy(message)
        rospy.sleep(2)
        for window in message.get('windows', []):
            if window.get('activity', '') == 'image':
                rospy.logerr('appending: {}'.format(window['assets'][0]))
                new_image_windows.append(window['assets'][0])
        feh_assets = self._get_feh_assets()
        rospy.logerr('new image windows: {}'.format(new_image_windows))
        for feh_asset in feh_assets:
            if feh_asset not in new_image_windows:
                assets_to_remove.append(feh_asset)
        if assets_to_remove:
            rospy.logerr('ASSETS TO REMOVE')
            if self.last_message == message:
                rospy.logerr('Directive 666')
                for image_proc in IMAGE_PROCS_TO_KILL:
                    subprocess.call(PARTIAL_KILLER_CMD + [image_proc])

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
    checker = ImageChecker()
    rospy.Subscriber('/director/scene', GenericMessage, checker.handle_director)
    rospy.spin()

if __name__ == '__main__':
    main()
