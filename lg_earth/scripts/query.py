#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from lg_common.helpers import get_params
from lg_earth import QueryWriter

if __name__ == '__main__':
    rospy.init_node('earth_query')

    query_file = get_params('~query_file', '/tmp/ge_queryfile')
    queue_length = get_params('~queue_length', 10)
    writer = QueryWriter(query_file, queue_length)

    rospy.Subscriber(
        '/earth/query/flyto_kml',
        String,
        writer.handle_flyto_kml
    )
    rospy.Subscriber(
        '/earth/query/flyto_pose_camera',
        Pose,
        writer.handle_flyto_pose_camera
    )
    rospy.Subscriber(
        '/earth/query/flyto_pose_lookat',
        Pose,
        writer.handle_flyto_pose_lookat
    )
    rospy.Subscriber(
        '/earth/query/search',
        String,
        writer.handle_search
    )
    rospy.Subscriber(
        '/earth/query/planet',
        String,
        writer.handle_planet
    )
    rospy.Subscriber(
        '/earth/query/tour',
        String,
        writer.handle_tour
    )

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
