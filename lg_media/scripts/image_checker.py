#!/usr/bin/env python
import json
import rospy
import subprocess
from lg_common.srv import USCSMessage

PGREP_CMD = ['pgrep', '-a', 'feh']
EGREP_CMD = ['egrep', '-o', 'http?://[^ ]+']
PARTIAL_KILLER_CMD = ['pkill', '-9', '-f']
LOOP_TIMEOUT = 5

class ImageChecker():
    def __init__(self, last_uscs_service):
        self.last_uscs_service = last_uscs_service
        self.last_uscs = ''
        self._keep_alive()

    def _keep_alive(self):
        image_procs_to_kill = ['image_viewer.py', 'feh', 'pqiv']
        while not rospy.is_shutdown():
            assets_to_remove = []
            current_image_assets = []
            self.last_uscs = json.loads(self.last_uscs_service().message)
            for window in self.last_uscs['windows']:
                if window['activity'] == 'image':
                    rospy.logerr('appending asset {}'.format(window['assets'][0]))
                    current_image_assets.append(window['assets'][0])

            rospy.logerr('current image assets: {}'.format(current_image_assets))
            feh_assets = self._get_feh_assets()
            if feh_assets:
                for feh_asset in feh_assets:
                    if feh_asset not in current_image_assets:
                        rospy.logerr('appending image: {} not in current image assets: {}'.format(feh_asset, current_image_assets))
                        assets_to_remove.append(feh_asset)
               
                if assets_to_remove:
                    rospy.logerr('checking if uscs is same')
                    if self.last_uscs == json.loads(self.last_uscs_service().message):
                        for image_proc in image_procs_to_kill:
                            rospy.logerr('partial killer cmd: {}'.format(PARTIAL_KILLER_CMD + [image_proc]))
                            subprocess.call(PARTIAL_KILLER_CMD + [image_proc])
            rospy.sleep(LOOP_TIMEOUT)

    def _get_feh_assets(self):
        feh_assets = ''
        pgrep = subprocess.Popen(PGREP_CMD, stdout=subprocess.PIPE)
        feh_proc = subprocess.Popen(EGREP_CMD, stdin=pgrep.stdout, stdout=subprocess.PIPE)
        feh_proc_assets = feh_proc.communicate()
        if feh_proc_assets[0]:
            rospy.logerr('feh assets {}'.format(feh_proc_assets[0]))
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
