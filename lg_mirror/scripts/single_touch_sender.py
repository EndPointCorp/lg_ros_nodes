#!/usr/bin/env python

import rospy

from lg_common.helpers import run_with_influx_exception_handler
from lg_mirror.msg import EvdevEvents, ScreenCoordinate
from lg_mirror.fake_touchscreen import MIN_X, MAX_X, MIN_Y, MAX_Y
from lg_mirror.touch_adapter import SingleTouchAdapter
from lg_mirror.touch_adapter import SingleTouchTransformer

NODE_NAME = 'single_touch_adapter'


def main():
    rospy.init_node(NODE_NAME)

    min_x = int(rospy.get_param('~min_x', MIN_X))
    max_x = int(rospy.get_param('~max_x', MAX_X))
    min_y = int(rospy.get_param('~min_y', MIN_Y))
    max_y = int(rospy.get_param('~max_y', MAX_Y))

    transformer = SingleTouchTransformer(min_x, max_x, min_y, max_y)

    coord_pub = rospy.Publisher(
        '/lg_mirror/fake_touch',
        ScreenCoordinate,
        queue_size=10
    )

    adapter = SingleTouchAdapter(coord_pub, transformer=transformer)

    rospy.Subscriber(
        '/lg_mirror/touch_events',
        EvdevEvents,
        adapter.handle_touch_events
    )

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
