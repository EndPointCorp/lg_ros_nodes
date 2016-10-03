#!/usr/bin/env python

import rospy
import os
from thread import start_new_thread
from lg_earth import Client
from lg_earth.client_config import get_config
from tempfile import gettempdir as systmp
from std_msgs.msg import String
from lg_earth import ViewsyncRelay
from geometry_msgs.msg import PoseStamped
from lg_common.msg import ApplicationState
from lg_common.helpers import check_www_dependency, x_available_or_raise, make_soft_relaunch_callback
from lg_earth.srv import ViewsyncState


def main():
    rospy.init_node('lg_earth')

    kmlsync_host = 'localhost'
    kmlsync_port = rospy.get_param('/kmlsync_server/port', 8765)
    global_dependency_timeout = rospy.get_param('/global_dependency_timeout', 15)
    depend_on_kmlsync = rospy.get_param('~depend_on_kmlsync', False)
    initial_state = rospy.get_param('~initial_state', 'VISIBLE')
    state_topic = rospy.get_param('~state_topic', '/earth/state')

    if os.environ.get("LG_LANG"):
        os.environ["LANG"] = os.environ["LG_LANG"]

    check_www_dependency(depend_on_kmlsync, kmlsync_host, kmlsync_port, 'kmlsync', global_dependency_timeout)

    x_available_or_raise(global_dependency_timeout)

    viewsync_port = None
    if rospy.get_param('~viewsync_send', False):
        viewsync = make_viewsync()
        viewsync_port = viewsync.listen_port

    instance = '_earth_instance_' + rospy.get_name().strip('/')
    tmpdir = os.path.normpath(systmp() + '/' + instance)
    config = get_config(tmpdir, instance, viewsync_port)
    # extend config with tmpdir and instance
    config = config + (tmpdir, instance)
    client = Client(config, initial_state=initial_state)

    rospy.Subscriber(state_topic, ApplicationState,
                     client.earth_proc.handle_state_msg)
    make_soft_relaunch_callback(client._handle_soft_relaunch, groups=["earth"])

    rospy.spin()


def make_viewsync():
    repeat_host = rospy.get_param('~viewsync_host', '10.42.42.255')
    repeat_port = rospy.get_param('~viewsync_port_new', 42000)

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

    viewsync_state_service = rospy.Service('/earth/viewsync/state', ViewsyncState, relay.get_last_state)

    start_new_thread(relay.run, ())
    return relay

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
