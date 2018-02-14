#!/usr/bin/env python

import sys
import rospy
from lg_volume_control import VolumeControlSlave


def main():
    rospy.init_node('volume_control_slave')

    rospy.wait_for_service('volume')
    volume_to_set = grab_master_volume()

    audio_sink = rospy.get_param('~default_sink', 0)

    volume_slave = VolumeControlSlave(audio_sink)
    volume.slave.set_volume(volume_to_set)

    rospy.Subscriber('/volume/level', UInt8, volume_slave.set_volume())

    rospy.spin()



def grab_master_volume():
    try:
        master_node_volume = rospy.ServiceProxy('volume', Volume)
        volume = master_volume()
    except rospy.ServiceException, e:
        rospy.logerr("Problem grabbing the master volume")

