#!/usr/bin/env python

import rospy
from lg_earth import Client
from lg_common.msg import ApplicationState


def main():
    rospy.init_node('lg_earth')

    client = Client()

    rospy.Subscriber('/earth/state', ApplicationState,
                     client.earth_proc.handle_state_msg)
    client.earth_proc.set_state(ApplicationState.VISIBLE)

    rospy.spin()

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
