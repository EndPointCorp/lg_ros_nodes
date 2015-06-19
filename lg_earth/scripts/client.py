#!/usr/bin/env python

import rospy
from lg_earth import Client


def main():
    rospy.init_node('lg_earth')

    client = Client()
    client.run()

    rospy.spin()

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
