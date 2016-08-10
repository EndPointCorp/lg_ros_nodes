#!/usr/bin/env python
"""
Just a ROS node load script.
Note that it can't be named 'lg_offliner', would
be getting Python ImportError.

"""


from lg_offliner import main

if __name__ == "__main__":
    main()
