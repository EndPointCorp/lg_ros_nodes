#!/usr/bin/env python

import rospy
from lg_earth import KMLSyncServer


def main():
    rospy.init_node('kmlsync_server', anonymous=True)

    KMLSyncServer().run()
    rospy.spin()

if __name__ == '__main__':
    main()
