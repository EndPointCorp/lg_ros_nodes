#!/usr/bin/env python3

import rospy
import os
from _thread import start_new_thread
from lg_earth import Client
from lg_earth.client_config import get_config
from tempfile import gettempdir as systmp
from std_msgs.msg import String
from lg_earth import ViewsyncRelay
from geometry_msgs.msg import PoseStamped
from lg_msg_defs.msg import ApplicationState
from lg_common.helpers import check_www_dependency, x_available_or_raise, make_soft_relaunch_callback, director_listener_earth_state
from lg_msg_defs.srv import ViewsyncState
from lg_common.helpers import run_with_influx_exception_handler
from time import sleep
from random import randint


NODE_NAME = 'lg_earth'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


def main():
    rospy.init_node(NODE_NAME)

    kmlsync_host = 'localhost'
    kmlsync_port = rospy.get_param('/kmlsync_server/port', 8765)
    global_dependency_timeout = rospy.get_param('/global_dependency_timeout', 15)
    depend_on_kmlsync = rospy.get_param('~depend_on_kmlsync', False)
    initial_state = rospy.get_param('~initial_state', 'VISIBLE')
    state_topic = rospy.get_param('~state_topic', '/earth/state')
    activity_list = rospy.get_param('~full_screen_activities', 'earth,cesium,mapbox,streetview,panovideo,panoviewer,unreal,unity,pannellum')

    state_pub = rospy.Publisher(state_topic, ApplicationState, queue_size=10)
    director_listener_earth_state(state_pub, activity_list.split(','))

    if os.environ.get("LG_LANG"):
        os.environ["LANG"] = os.environ["LG_LANG"]

    check_www_dependency(depend_on_kmlsync, kmlsync_host, kmlsync_port, 'kmlsync', global_dependency_timeout)

    x_available_or_raise(global_dependency_timeout)

    viewsync_port = None
    if rospy.get_param('~viewsync_send', False):
        viewsync = make_viewsync()
        viewsync_port = viewsync.listen_port
    random_stagger = rospy.get_param('~staggered', False)
    if random_stagger:
        random_sleep_length = randint(1, 10)
        logger.debug("Random sleep length: {}".format(random_sleep_length))
        sleep(random_sleep_length)

    instance = '_earth_instance_' + rospy.get_name().strip('/')
    tmpdir = os.path.normpath(systmp() + '/' + instance)
    config = get_config(tmpdir, instance, viewsync_port)
    # extend config with tmpdir and instance
    config = config + (tmpdir, instance)
    client = Client(config, initial_state=initial_state)

    rospy.Subscriber(state_topic, ApplicationState,
                     client.earth_proc.handle_state_msg)
    if random_stagger:
        make_soft_relaunch_callback(client._handle_staggered_soft_relaunch, groups=["earth"])
    else:
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
    run_with_influx_exception_handler(main, NODE_NAME)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
