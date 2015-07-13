#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from lg_pano import PanoMaster

if __name__ == '__main__':
    rospy.init_node('pano_master')
    pm = PanoMaster()
    rospy.Subscriber(
        '/spacenav/twist',
        Twist,
        pm.spacenavTwist
    )

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
