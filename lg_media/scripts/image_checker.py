#!/usr/bin/env python
import json
import rospy
import subprocess
from lg_common.srv import USCSMessage

PGREP_CMD = ['pgrep', '-a', 'feh']
EGREP_CMD = ['egrep', '-o', 'http?://[^ ]+']
PARTIAL_KILLER_CMD = ['pkill', '-9', '-f']
IMAGE_PROCS_TO_KILL = ['image_viewer.py', 'feh', 'pqiv']
LOOP_TIMEOUT = 5


class ImageChecker():
    def __init__(self, last_uscs_service):
        self.last_uscs_service = last_uscs_service
        self.last_uscs = ''
        self._keep_alive()

    def _keep_alive(self):
        while not rospy.is_shutdown():
            assets_to_remove = []
            current_image_assets = []
            self.last_uscs = json.loads(self.last_uscs_service().message)
            for window in self.last_uscs['windows']:
                if window['activity'] == 'image':
                    current_image_assets.append(window['assets'][0])

            feh_assets = self._get_feh_assets()
            if feh_assets:
                for feh_asset in feh_assets:
                    if feh_asset not in current_image_assets:
                        assets_to_remove.append(feh_asset)

                if assets_to_remove:
                    if self.last_uscs == json.loads(self.last_uscs_service().message):
                        for image_proc in image_procs_to_kill:
                            subprocess.call(PARTIAL_KILLER_CMD + [image_proc])
            rospy.sleep(LOOP_TIMEOUT)

    def _get_feh_assets(self):
        feh_assets = ''
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
    rospy.spin()

if __name__ == '__main__':
    main()
