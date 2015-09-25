#!/usr/bin/env python

import rospy
from lg_earth import Client
from lg_common.msg import ApplicationState
from lg_common.helpers import DependencyException


def main():
    rospy.init_node('lg_earth')

    kmlsync_host = 'localhost'
    kmlsync_port = rospy.get_param('/kmlsync_server/port', 8765)
    kmlsync_timeout = rospy.get_param('/global_dependency_timeout', 15)

    if not depend_on_service(kmlsync_host, kmlsync_port, 'kmlsync', kmlsync_timeout):
        msg = "Service: %s didnt get accessible within %s seconds" % ('kmlsync', kmlsync_timeout)
        rospy.lofatal(msg)
        raise DependencyException(msg)
    else:
        rospy.loginfo("KMLsync server is active - initializing Earth")

    client = Client()

    rospy.Subscriber('/earth/state', ApplicationState,
                     client.earth_proc.handle_state_msg)
    client.earth_proc.set_state(ApplicationState.VISIBLE)

    rospy.spin()

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
