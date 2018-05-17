#!/usr/bin/env python

import sys
import rospy
from lg_volume_control import VolumeControlSlave
from lg_volume_control.srv import Volume
from std_msgs.msg import Int8


def main():
    rospy.init_node('volume_control_slave')

    rospy.wait_for_service('volume')
    volume_to_set = grab_master_volume()

    audio_sink = rospy.get_param('~default_sink', "@DEFAULT_SINK@")

    volume_slave = VolumeControlSlave(audio_sink)
    volume_slave.set_volume(volume_to_set)

    rospy.Subscriber('/volume/level', Int8, volume_slave.set_volume_topic_handler)

    rospy.spin()


def grab_master_volume():
    try:
        master_node_volume = rospy.ServiceProxy('volume', Volume)
        rospy.sleep(1)
        volume = master_node_volume()
        rospy.logdebug("got volume {} from master".format(volume))
        return volume.volume
    except rospy.ServiceException, e:
        rospy.logerr("Problem grabbing the master volume")
        return None


if __name__ == '__main__':
    main()
