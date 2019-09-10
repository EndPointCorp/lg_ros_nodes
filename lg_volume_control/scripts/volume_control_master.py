#!/usr/bin/env python3

import rospy
from lg_volume_control.srv import Volume
from lg_volume_control import VolumeControlMaster
from std_msgs.msg import Int8, UInt8


def main():
    rospy.init_node('volume_control_master')

    level_change = rospy.Publisher('/volume/level', UInt8, latch=True, queue_size=1)
    default_volume = rospy.get_param('~default_volume', 50)

    volume_controller = VolumeControlMaster(level_change, default_volume)

    rospy.Subscriber('/volume/increment', Int8, volume_controller.handle_change_volume)

    rospy.Service('volume', Volume, volume_controller.show_volume)

    rospy.spin()


if __name__ == '__main__':
    main()
