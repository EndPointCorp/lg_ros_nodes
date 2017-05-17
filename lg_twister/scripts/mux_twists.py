#!/usr/bin/env python

from functools import partial
import math
from threading import Lock

import rospy
from geometry_msgs.msg import Twist

NODE_NAME = 'mux_twists'

DEFAULT_TICK_RATE = 65.0
DEFAULT_AXIS_LIMIT = math.sqrt(2) / 2
DEFAULT_AGE_LIMIT = 1.0


def clamp(val, lo, hi):
    return min(max(val, lo), hi)


def clamp_twist(twist, lo, hi):
    twist.linear.x = clamp(twist.linear.x, lo, hi)
    twist.linear.y = clamp(twist.linear.y, lo, hi)
    twist.linear.z = clamp(twist.linear.z, lo, hi)
    twist.angular.x = clamp(twist.angular.x, lo, hi)
    twist.angular.y = clamp(twist.angular.y, lo, hi)
    twist.angular.z = clamp(twist.angular.z, lo, hi)


class TwistMuxer:
    def __init__(self, twist_pub, axis_limit, age_limit):
        self._lock = Lock()

        self.twist_pub = twist_pub
        self.axis_limit = axis_limit
        self.age_limit = rospy.Duration(age_limit)

        self.samples = {}
        self.sample_stamps = {}

    def handle_twist(self, topic, twist):
        with self._lock:
            self._handle_twist(topic, twist)

    def _handle_twist(self, topic, twist):
        self.samples[topic] = twist
        self.sample_stamps[topic] = rospy.Time.now()

    def tick(self, tev):
        with self._lock:
            self._tick(tev)

    def _tick(self, tev):
        t = rospy.Time.now()

        result = Twist()
        for topic in self.samples.keys():
            stamp = self.sample_stamps[topic]
            if t - stamp > self.age_limit:
                continue

            twist = self.samples[topic]
            result.linear.x += twist.linear.x
            result.linear.y += twist.linear.y
            result.linear.z += twist.linear.z
            result.angular.x += twist.angular.x
            result.angular.y += twist.angular.y
            result.angular.z += twist.angular.z

        clamp_twist(result, -self.axis_limit, self.axis_limit)

        self.twist_pub.publish(result)


def main():
    rospy.init_node(NODE_NAME)

    tick_rate = float(rospy.get_param('~tick_rate', DEFAULT_TICK_RATE))
    sources = [
        s.strip() for s in rospy.get_param('~sources').split(',')
    ]
    axis_limit = float(rospy.get_param('~axis_limit', DEFAULT_AXIS_LIMIT))
    age_limit = float(rospy.get_param('~age_limit', DEFAULT_AGE_LIMIT))

    twist_pub = rospy.Publisher('/lg_twister/twist', Twist, queue_size=10)

    muxer = TwistMuxer(twist_pub, axis_limit, age_limit)

    for source in sources:
        handler = partial(muxer.handle_twist, source)
        rospy.Subscriber(source, Twist, handler)

    rospy.Timer(rospy.Duration(1.0 / tick_rate), muxer.tick)

    rospy.spin()


if __name__ == '__main__':
    main()
