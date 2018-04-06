#!/usr/bin/env python

import threading
import rospy
import commands


class VolumeControlMaster:
    def __init__(self, level_change_pub, default_volume=50, scale=5):
        self.scale = scale
        self.default_volume = default_volume
        self.current_volume = -1
        volume = self.clamp(self.default_volume, 0, 100)
        self.level_change_pub = level_change_pub
        self.set_volume(volume)

    def set_volume(self, volume):
        volume = self.clamp(volume)
        if volume == self.current_volume:
            rospy.loginfo("VolumeControlMaster: No change to volume level")
            return
        self.current_volume = volume
        rospy.loginfo(
            "VolumeControlMaster: Setting volume on VolumeControlSlaves to {}%".format(volume))
        self.level_change_pub.publish(volume)

    def clamp(self, value, _min=0, _max=100):
        return max(min(value, _max), _min)

    def handle_change_volume(self, msg):
        increment = int(msg.data) * self.scale
        self.set_volume(self.current_volume + increment)

    def show_volume(self, *args, **kwargs):
        return self.current_volume


class VolumeControlSlave:
    def __init__(self, sink='0'):
        self.sink = sink
        self._lock = threading.Lock()

    def set_volume_topic_handler(self, msg):
        self.set_volume(msg.data)

    def set_volume(self, volume):
        rospy.loginfo("about to grab the lock...")
        cmd = "pactl set-sink-volume {} {}%".format(self.sink, volume)
        with self._lock:
            rospy.loginfo("running command: {}".format(cmd))
            status, output = commands.getstatusoutput(cmd)
            rospy.loginfo("output {} status {}".format(output, status))
