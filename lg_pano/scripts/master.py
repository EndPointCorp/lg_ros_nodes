#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from lg_common.msg import ApplicationState
from std_msgs.msg import String
from lg_pano import PanoMaster

if __name__ == '__main__':
    rospy.init_node('pano_master')

    # Does it really matter what the queue size is?
    pov  = rospy.Publisher('/panoviewer/pov',  Vector3, queue_size=10)
    pano = rospy.Publisher('/panoviewer/pano', String,  queue_size=10)

    pm = PanoMaster(pov, pano)
    # XXX The issue says we need to listen to this, but what's the msg type?
    # rospy.Subscriber(
    #     '/panoviewer/state',
    # )
    rospy.Subscriber(
        '/panoviewer/init',
        String,
        pm.initApp
    )
    rospy.Subscriber(
        '/spacenav/twist',
        Twist,
        pm.spacenavTwist
    )

    rospy.Subscriber(
        '/panoviewer/state',
        ApplicationState,
        pm.handle_state
    )

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
