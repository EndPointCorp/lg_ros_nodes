#!/usr/bin/env python

import rospy

from lg_common.helpers import run_with_influx_exception_handler
from lg_mirror import FakeTouchscreen
from lg_mirror.msgs import ScreenCoordinate

NODE_NAME = 'Touchscreen Emulator'


def main():
    rospy.init_node(NODE_NAME)

    touchscreen = FakeTouchscreen()

    rospy.Subscriber('/lg_mirror/fake_touch',
                     ScreenCoordinate,
                     touchscreen.touch)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
