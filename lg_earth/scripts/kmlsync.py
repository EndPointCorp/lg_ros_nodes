#!/usr/bin/env python

import rospy
from lg_common import webapp
from lg_earth import KMLSyncServer


def main():
    rospy.init_node('kmlsync_server', anonymous=True)

    host = rospy.get_param('kmlsync_listen_host', '127.0.0.1')
    port = rospy.get_param('kmlsync_listen_port', 8765)

    server = KMLSyncServer(host=host, port=port)

    rospy.spin()

if __name__ == '__main__':
    main()
