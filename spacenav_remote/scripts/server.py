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

    joy_topic = topic + '/joy'
    twist_topic = topic + '/twist'

    joy_pub = rospy.Publisher(joy_topic, Joy, queue_size=10)
    twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=10)

    print "Publish joy to: " + joy_topic
    print "Publish twist to: " + twist_topic

    def handler(data):

        try:
            recived = byteify(json.loads(data))

            # Send joystic data
            joy = Joy()
            joy.axes = (recived.trans + recived.rot)
            joy_pub.publish(joy)

            # Send twists data
            twist = Twist()
            twist.angular = recived.rot
            twist.linear = recived.trans
            twist_pub.publish(twist)
        except AttributeError, e:
            print e

    server = SpacenavRemote(handler=handler, port=port)
    server.fork_and_run()

    def shutdown_server():
        server.shutdown()

    rospy.on_shutdown(shutdown_server)
    rospy.spin()

def byteify(input):
    if isinstance(input, dict):
        return {byteify(key): byteify(value)
                for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [byteify(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

if __name__ == "__main__":
    main()
