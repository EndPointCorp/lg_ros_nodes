#!/usr/bin/env python3

import rospy

from lg_earth import ViewsyncRelay
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from lg_earth.srv import ViewsyncState
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'earth_viewsync_relay'


def main():
    rospy.init_node(NODE_NAME)

    listen_host = rospy.get_param('~listen_host', '127.0.0.1')
    listen_port = rospy.get_param('~listen_port', 42000)
    repeat_host = rospy.get_param('~repeat_host', '<broadcast>')
    repeat_port = rospy.get_param('~repeat_port', 42000)

    pose_pub = rospy.Publisher(
        '/earth/pose', PoseStamped, queue_size=3
    )
    planet_pub = rospy.Publisher(
        '/earth/planet', String, queue_size=3
    )

    relay = ViewsyncRelay(
        repeat_addr=(repeat_host, repeat_port),
        pose_pub=pose_pub,
        planet_pub=planet_pub
    )

    relay.run()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
