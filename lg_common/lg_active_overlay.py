#!/usr/bin/env python3

import os
os.environ['DISPLAY'] = ':0'
os.environ['PS1'] = 'foo'
import time
import subprocess
import rospy
from lg_msg_defs.msg import MediaOverlays
from std_msgs.msg import Bool

active_stream = False
def check_overlay_state(msg):
    global active_stream
    if "[]" in str(msg):
        active_stream = False
    else:
        active_stream = True

rospy.init_node('lg_overlay_listener')
rospy.Subscriber('/media_overlays', MediaOverlays, check_overlay_state)
active_publisher = rospy.Publisher('/touchscreen/touch', Bool, queue_size=2)
rate = rospy.Rate(0.2)

while not rospy.is_shutdown():
    if active_stream:
        active_publisher.publish(True)
        active_publisher.publish(False)
        rate.sleep()

