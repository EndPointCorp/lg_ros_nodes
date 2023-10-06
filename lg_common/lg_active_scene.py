#!/usr/bin/env python3

import os
os.environ['DISPLAY'] = ':0'
os.environ['PS1'] = 'foo'
import time
import subprocess
import rospy
import json
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from std_msgs.msg import Bool

active_timer = None
sys_active = True

def is_active(is_it):
    """ callback for activity/active listener """
    global sys_active
    sys_active = is_it.data

def clear_timer(what):
    """ callback for timer to clear itself if no scene was played """
    global active_timer
    if active_timer is not None:
        active_timer.shutdown()
        active_timer = None

def check_scene_duration(data):
    """ callback for director/scene listener """
    global active_timer
    if active_timer is not None:  # shutdown previous timer if new scene is played
        active_timer.shutdown()
        active_timer = None
    if not sys_active:  # only keep system active, if inactive do nothing (attract loop plays on inactive)
        return
    try:
        msg = json.loads(data.message)
        duration = msg['duration']
        #print(duration)
        ## TODO if duration == 0/infinity or 666 might need a param value
        if int(duration) > 0:  # set a self clearing timer to keep system active while it exists
            active_timer = rospy.Timer(rospy.Duration(duration), clear_timer, oneshot=True)
    except:
        #active_timer = None
        pass

rospy.init_node('lg_active_scene')
rospy.Subscriber('/activity/active', Bool, is_active)
rospy.Subscriber('/director/scene', GenericMessage, check_scene_duration)
active_publisher = rospy.Publisher('/touchscreen/touch', Bool, queue_size=2)
rate = rospy.Rate(0.2)

while not rospy.is_shutdown():
    if active_timer:
        active_publisher.publish(True)
        active_publisher.publish(False)
        rate.sleep()
