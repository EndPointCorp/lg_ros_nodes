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
from lg_common.helpers import DependencyException
from lg_common.helpers import dependency_available, x_available, make_soft_relaunch_callback


def main():
    rospy.init_node('lg_earth')

    kmlsync_host = 'localhost'
    kmlsync_port = rospy.get_param('/kmlsync_server/port', 8765)
    kmlsync_timeout = rospy.get_param('/global_dependency_timeout', 15)
    depend_on_kmlsync = rospy.get_param('~depend_on_kmlsync', False)
    initial_state = rospy.get_param('~initial_state', 'VISIBLE')
    state_topic = rospy.get_param('~state_topic', '/earth/state')

    if os.environ.get("LG_LANG"):
        os.environ["LANG"] = os.environ["LG_LANG"]

    if depend_on_kmlsync:
        rospy.loginfo("Waiting for KMLSync to become available")
        if not dependency_available(kmlsync_host, kmlsync_port, 'kmlsync', kmlsync_timeout):
            msg = "Service: %s hasn't become accessible within %s seconds" % ('kmlsync', kmlsync_timeout)
            rospy.logfatal(msg)
            raise DependencyException(msg)
        else:
            rospy.loginfo("KMLSync available - continuing initialization")

    x_timeout = rospy.get_param("/global_dependency_timeout", 15)
    if x_available(x_timeout):
        rospy.loginfo("X available")
    else:
        msg = "X server is not available"
        rospy.logfatal(msg)
        raise DependencyException(msg)

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

    start_new_thread(relay.run, ())
    return relay

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
