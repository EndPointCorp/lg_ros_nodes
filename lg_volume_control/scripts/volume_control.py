#!/usr/bin/env python

import rospy
from lg_volume_control import VolumeControl
from std_msgs.msg import UInt8, Int8


def main():
    rospy.init_node('volume_control')

    level_change = rospy.Publisher('/volume/level', UInt8, latch=True, queue_size=1)
    default_sink = rospy.get_param('~default_sink', '@DEFAULT_SINK@')

    volume_controller = VolumeControl(level_change_pub=level_change, default_sink=default_sink)

    rospy.Subscriber('/volume/increment', Int8, volume_controller.handle_volume_change_request)

    rospy.spin()

if __name__ == '__main__':
    main()
