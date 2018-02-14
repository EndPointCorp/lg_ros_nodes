#!/usr/bin/env python

import threading

class VolumeControlMaster:
    def __init__(self, level_change_pub, default_volume=50, scale=5):
        self.scale = scale
        self.default_volume = default_volume
        self.current_volume = -1
        volume = self.clamp(default, 0, 100)
        self.level_change_pub = level_change_pub
        self.set_volume(volume)

    def set_volume(self, volume):
        volume = self.clamp(volume)
        if volume == self.current_volume:
            return
        self.level_change_pub(volume)

    def clamp(self, value, _min=0, _max=100):
        return max(min(value, _max), _min)

    def handle_change_volume(self, msg):
        increment = int(msg.data) * self.scale
        self.set_volume(self.current_volume + increment)

    def show_volume(self):
        return self.current_volume

class VolumeControlSlave:
    def __init__(self, sink='0'):
        self.sink = sink
        self._lock = threading.Lock()

    def set_volume(self, volume):
        with self._lock():
            rospy.loginfo("running command: pactl set-sink-volume {} {}%%".format(self.sink, volume))
            try:
                commands.getstatusouput("pactl set-sink-volume {} {}%%".format(self.sink, volume))
            except:
                rospy.logerr("Error while setting the volume. Pulese audio might not be supported. Try running 'patctl list sinks' on this machine")
