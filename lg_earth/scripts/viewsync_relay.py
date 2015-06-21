#!/usr/bin/env python

import rospy

from lg_earth import ViewsyncRelay
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('earth_viewsync_relay')

    listen_host = rospy.get_param('~listen_host', '127.0.0.1')
    listen_port = rospy.get_param('~listen_port', 42000)
    repeat_host = rospy.get_param('~repeat_host', '<broadcast>')
    repeat_port = rospy.get_param('~repeat_port', 42000)

    pose_pub = rospy.Publisher(
        '/earth/pose', PoseStamped, queue_size=3
    )

    relay = ViewsyncRelay(
        listen_addr=(listen_host, listen_port),
        repeat_addr=(repeat_host, repeat_port),
        pose_pub=pose_pub
    )

    relay.run()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
