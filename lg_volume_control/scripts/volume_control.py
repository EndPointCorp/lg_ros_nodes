#!/usr/bin/env python

import rospy
import commands
from std_msgs.msg import UInt8, Int8


class MockPub:
    def publish(self, *args, **kwargs):
        pass

class VolumeControl:
    def __init__(self, level_change_pub=MockPub(), default_sink='0', default=50, scale=5):
        self.scale = scale
        self.default = default
        self.current_volume = -1
        self.sink = default_sink
        volume = self.clamp(self.grab_current_volume(), default/2, default)
        self.level_change_pub = level_change_pub

        self.set_volume(volume)

    def handle_volume_change_request(self, msg):
        increment = int(msg.data) * self.scale
        self.set_volume(self.current_volume + increment)

    def grab_current_volume(self):
        volume = int(commands.getoutput("pactl list sinks | grep '^[[:space:]]Volume:'").split('\n')[int(self.sink)].split(' ')[-1].split('%')[0])
        print("current volume is %s" % volume)
        return volume

    def set_volume(self, volume):
        print("setting volume to %s" % volume)
        volume = self.clamp(volume)
        print("setting volume to %s" % volume)
        if volume == self.current_volume:
            return

        print("running command: pactl set-sink-volume %s %s%%" % (self.sink, volume))
        commands.getstatusoutput("pactl set-sink-volume %s %s%%" % (self.sink, volume))

        self.current_volume = volume
        self.level_change_pub.publish(volume)


    def clamp(self, value, _min=0, _max=100):
        # clamps value between _max and _min
        return max(min(value, _max), _min)

def main():
    rospy.init_node('volume_control')

    level_change = rospy.Publisher('/volume/level', UInt8, latch=True, queue_size=1)
    default_sink = rospy.get_param('~default_sink', '0')
    print("default_sink is %s" % default_sink)

    volume_controller = VolumeControl(level_change_pub=level_change, default_sink=default_sink)

    rospy.Subscriber('/volume/increment', Int8, volume_controller.handle_volume_change_request)

    rospy.spin()

if __name__ == '__main__':
    main()
