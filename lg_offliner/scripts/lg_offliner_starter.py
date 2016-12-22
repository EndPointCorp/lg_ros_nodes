#!/usr/bin/env python
"""
Just a ROS node load script.
Note that it can't be named 'lg_offliner', would
be getting Python ImportError.

"""

NODE_NAME = 'lg_offliner'
from lg_offliner import main
from lg_common.helpers import run_with_influx_exception_handler

if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
