#!/usr/bin/env python3

import os
os.environ['DISPLAY'] = ':0'
os.environ['PS1'] = 'foo'
import rospy
import json
from interactivespaces_msgs.msg import GenericMessage
import subprocess
import time


def toggle_layer(msg):
    print("check msg", msg)
    try:
        json_msg = json.loads(msg.message)
        if json_msg:
            for layer in json_msg.keys():
                if json_msg[layer]:
                    #if layer in ["borders", "roads", "terrain", "buildings"]:
                    print('Received %s toggle command' % layer)
                    subprocess.run(['/home/lg/python_scripts/toggle_layer.py', layer])
                    time.sleep(5)
                    #else:
                    #    print("Received a not enabled command:", layer)
    except Exception as e:
        rospy.logerr("Invalid message: %s" % json_msg, "ERROR:  ", str(e))
        return

print("loaded py, initializing node lg_toggles...")
rospy.init_node('lg_toggles')

while not rospy.is_shutdown():
    try:
        rospy.Subscriber('/earth_layer', GenericMessage, toggle_layer)
        break
    except Exception as e:
        rospy.logerr("Error setting up subscriber: %s. Retrying in 10 seconds..." % str(e))
        time.sleep(10)

print("~\--/~  spiiiin")
rospy.spin()

