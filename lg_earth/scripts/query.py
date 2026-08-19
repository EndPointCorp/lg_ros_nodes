#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from lg_earth import QueryWriter
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'earth_query'


def main():
    rospy.init_node(NODE_NAME)
    query_file = rospy.get_param('~query_file', '/tmp/ge_queryfile')
    queue_length = rospy.get_param('~queue_length', 10)
    writer = QueryWriter(query_file, queue_length)
    rospy.on_shutdown(writer.shutdown)

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


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
