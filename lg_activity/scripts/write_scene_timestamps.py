#!/usr/bin/env python3

import os
import time
import json
import rospy
from interactivespaces_msgs.msg import GenericMessage

def save_scene_played_time(data):
    """Callback for /director/scene listener, updates the played time for the scene."""
    #input_file_path = '/media/videos/scene_times.json'
    json_file_path = '/home/lg/scene_times.json'
    if os.path.exists(json_file_path):
        with open(json_file_path, 'r') as file:
            scene_times = json.load(file)
    else:
        scene_times = [] 
    try:
        message_dict = json.loads(data.message)
    except:
        print("failed to load message")
        return

    slug = None
    if message_dict:
        slug = message_dict.get('slug', None)
    else:
        print(data)
        return

    if slug:
        timestamp = time.time()
        scene_times += [[timestamp, slug]]

        with open(json_file_path, 'w') as file:
            json.dump(scene_times, file, indent=4)


if __name__ == "__main__":
    rospy.init_node('lg_active_scene')
    rospy.Subscriber('/director/scene', GenericMessage, save_scene_played_time)
    rospy.spin()

