#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, Quaternion, Twist
from std_msgs.msg import String
from math import atan2, cos, sin, pi
from lg_sv import StreetviewServer


# spacenav_node -> /spacenav/twist -> handle_spacenav_msg:
# 1. change pov based on rotational axes -> /streetview/pov
# 2. check for movement -> /streetview/panoid

# /streetview/location -> handle_location_msg:
# 1. query api, publish -> /streetview/panoid
# low priority

# /streetview/metadata -> handle_metadata_msg:
# 1. update self.metadata


DEFAULT_TILT_MIN = -80
DEFAULT_TILT_MAX = 80
DEFAULT_NAV_SENSITIVITY = 1.0


def main():
    rospy.init_node('streetview_server')
    location_pub = rospy.Publisher('/streetview/location',
                              Pose2D, queue_size=1)
    panoid_pub = rospy.Publisher('/streetview/panoid',
                              String, queue_size=1)
    pov_pub = rospy.Publisher('/streetview/pov',
                              Quaternion, queue_size=2)

    tilt_min = rospy.get_param('~tilt_min', DEFAULT_TILT_MIN)
    tilt_max = rospy.get_param('~tilt_max', DEFAULT_TILT_MAX)
    nav_sensitivity = rospy.get_param('~nav_sensitivity', DEFAULT_NAV_SENSITIVITY)

    server = StreetviewServer(location_pub, panoid_pub, pov_pub, tilt_min, tilt_max, nav_sensitivity)

    rospy.Subscriber('/streetview/location', Pose2D,
                     server.handle_location_msg)
    rospy.Subscriber('/streetview/metadata', String,
                     server.handle_metadata_msg)
    rospy.Subscriber('/streetview/panoid', String,
                     server.handle_panoid_msg)
    rospy.Subscriber('/streetview/pov', Quaternion,
                     server.handle_pov_msg)
    rospy.Subscriber('/spacenav/twist', Twist,
                     server.handle_spacenav_msg)
    rospy.spin()

if __name__ == '__main__':
	main()
