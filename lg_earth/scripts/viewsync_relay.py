#!/usr/bin/env python

import rospy

from lg_earth import ViewsyncRelay
from lg_common.helpers import get_params
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('earth_viewsync_relay')

    listen_host = get_params('~listen_host', '127.0.0.1')
    listen_port = get_params('~listen_port', 42000)
    repeat_host = get_params('~repeat_host', '<broadcast>')
    repeat_port = get_params('~repeat_port', 42000)

    pose_pub = rospy.Publisher(
        '/earth/pose', PoseStamped, queue_size=3
    )
    planet_pub = rospy.Publisher(
        '/earth/planet', String, queue_size=3
    )

    relay = ViewsyncRelay(
        listen_addr=(listen_host, listen_port),
        repeat_addr=(repeat_host, repeat_port),
        pose_pub=pose_pub,
        planet_pub=planet_pub
    )

    relay.run()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
