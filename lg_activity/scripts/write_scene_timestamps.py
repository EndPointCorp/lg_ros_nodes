#!/usr/bin/env python3

import os
import time
import json
import rospy
from interactivespaces_msgs.msg import GenericMessage


IGNORED_SCENE_SLUGS = ["auto_generated_sv_scene"]

def save_scene_played_time(data):
    """Callback for /director/scene listener, updates the played time for the scene."""
    #input_file_path = '/media/videos/scene_times.json'
    scene_times_path = '/mnt/videos/scene_times.json'
    scene_infos_path = '/mnt/videos/scene_infos.json'
    scene_times = []
    if os.path.exists(scene_times_path):
        try:
            with open(scene_times_path, 'r') as times_file:
                scene_times = json.load(times_file)
        except:
            pass
    try:
        message_dict = json.loads(data.message)
    except:
        print("failed to load message")
        return

    slug = None
    if message_dict:
        slug = message_dict.get('slug', None)
    else:
        print("failed to load: ", data)
        return

    if not slug or slug in IGNORED_SCENE_SLUGS:
        print("nothing to save on this message")
        return

    try:
        timestamp = time.time()
        scene_times += [[timestamp, slug]]

        with open(scene_times_path, 'w') as times_file:
            json.dump(scene_times[-20000:], times_file, indent=4)

    except Exception as e:
        print(f"something failed updating played time for scene with slug: {slug}")

    scene_duration = message_dict.get('duration', '0')
    scene_name = message_dict.get('scene_name', None)
    scene_infos = {}
    if os.path.exists(scene_infos_path):
        try:
            with open(scene_infos_path, 'r') as info_file:
                scene_infos = json.load(info_file)
        except:
            pass

    if scene_infos.get(slug, {}).get("scene_name", None) != scene_name or scene_infos.get(slug, {}).get("duration", "0") != scene_duration:
        scene_infos[slug] = {}
        scene_infos[slug]["scene_name"] = scene_name
        scene_infos[slug]["duration"] = scene_duration

        with open(scene_infos_path, 'w') as infos_file:
            json.dump(dict(list(scene_infos.items())[-2000:]), infos_file, indent=4)


if __name__ == "__main__":
    rospy.init_node('lg_active_scene')
    rospy.Subscriber('/director/scene', GenericMessage, save_scene_played_time)
    rospy.spin()

