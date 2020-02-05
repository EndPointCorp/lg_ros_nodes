#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

from lg_earth import Toggle3d

from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'earth_3d_layer_service'


def main():
    # We need that on 42-a as well as on 42-b
    rospy.init_node(NODE_NAME, anonymous=True)

    s = Toggle3d()

    pub = rospy.Publisher('/earth/3d_layer/state', Bool, queue_size=1, latch=True)
    s.set_on_change_listener(lambda state: pub.publish(state))

    rospy.Subscriber('/earth/3d_layer/set',
                     Bool,
                     lambda msg: s.set_layer_state(msg.data))

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
