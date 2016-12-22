#!/usr/bin/env python

import rospy
from lg_stats import main
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'lg_stats'

if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
