#!/usr/bin/env python3

import subprocess, rospy, os
from std_msgs.msg import String

TARGET_WINDOW="earth ec"  # change to regex if needed

def hit_key(msg):
    key = msg.data or 'u'
    try:
        win_id = subprocess.check_output(['xdotool', 'search', '--name', TARGET_WINDOW]).split()[0].decode()
        #subprocess.check_call(['xdotool', 'windowfocus', '--sync', win_id, 'mousemove', '--window', win_id, '10', '10', 'click', '1', 'key', key, 'key', key])
        subprocess.check_call(['xdotool', 'windowfocus', '--sync', win_id, 'mousemove', '--window', win_id, '10', '10', 'click', '1', 'key', key, 'key', key])
        #subprocess.check_call(['xdotool', 'windowfocus', '--sync', win_id, 'key', key])
        rospy.loginfo(f"Sent key {key} to window {win_id}")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"xdotool failure: {e}")

def main():
    rospy.init_node('keystroke_listener')
    rospy.Subscriber('/google_earth_center/keystroke', String, hit_key)
    rospy.spin()

if __name__ == '__main__':
    main()
