#!/bin/bash
set -e

# setup ros environment
source "/home/galadmin/catkin_ws/catkin/devel/setup.bash"
spacenavd
exec "$@"
