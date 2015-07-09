#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from lg_pointcloud import PointcloudMaster

if __name__ == '__main__':
    rospy.init_node('pointcloud_master')
    pcm = PointcloudMaster()
    rospy.Subscriber(
        '/spacenav/twist',
        Twist,
        pcm.spacenavTwist
    )

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

