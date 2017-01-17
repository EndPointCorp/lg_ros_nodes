#! /usr/bin/env python

import json
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from spacenav_remote import SpacenavRemote

NODE_NAME = 'spacenav_remote'

def main():
    rospy.init_node(NODE_NAME)
    topic = rospy.get_param('~topic', '/spacenav')
    port = rospy.get_param('~listen_port', 6564)
    verbose = rospy.get_param('~verbose', False)

    joy_pub = rospy.Publisher(topic + '/joy', Joy, queue_size=10)
    twist_pub = rospy.Publisher(topic + '/twist', Twist, queue_size=10)

    def handler(data):
        if verbose:
            print data

        try:
            recived = json.loads(data)

            # Send joystic data
            joy = Joy()
            joy.axes = recived.trans
            joy_pub.publish(joy)

            # Send twists data
            twist = Twist()
            twist.angular = recived.rot
            twist.linear = recived.trans
            twist_pub.publish(twist)
        except AttributeError:
            pass

    server = SpacenavRemote(handler=handler, port=port)
    server.fork_and_run()

    def shutdown_server():
        server.shutdown()

    rospy.on_shutdown(shutdown_server)
    rospy.spin()


if __name__ == "__main__":
    main()
