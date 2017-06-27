import rospy
import commands


class MockPub:
    def publish(self, *args, **kwargs):
        pass


class VolumeControl:
    def __init__(self, level_change_pub=MockPub(), default_sink='0', default=50, scale=5):
        self.scale = scale
        self.default = default
        self.current_volume = -1
        self.sink = default_sink
        volume = self.clamp(default, 0, 100)
        rospy.info("default is {} and volume is {}".format(default, volume))
        self.level_change_pub = level_change_pub

        self.set_volume(volume)

    def handle_volume_change_request(self, msg):
        increment = int(msg.data) * self.scale
        self.set_volume(self.current_volume + increment)

    def grab_current_volume(self):
        try:
            volume = int(commands.getoutput("pactl list sinks | grep '^[[:space:]]Volume:'").split('\n')[int(self.sink)].split(' ')[-1].split('%')[0])
        except:
            rospy.logerr("Error while grabbing the current volume. Pulse audio might not be supported. Try running 'pactl list sinks' on this machine")
            volume = self.default
        rospy.loginfo("current volume is %s" % volume)
        return volume

    def set_volume(self, volume):
        volume = self.clamp(volume)
        if volume == self.current_volume:
            return

        rospy.loginfo("running command: pactl set-sink-volume %s %s%%" % (self.sink, volume))
        try:
            commands.getstatusoutput("pactl set-sink-volume %s %s%%" % (self.sink, volume))
        except:
            rospy.logerr("Error while setting the volume. Pulse audio might not be supported. Try running 'pactl list sinks' on this machine")

        self.current_volume = volume
        self.level_change_pub.publish(volume)

    def clamp(self, value, _min=0, _max=100):
        # clamps value between _max and _min
        return max(min(value, _max), _min)
