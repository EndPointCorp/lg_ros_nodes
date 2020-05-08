#!/bin/bash

set -e
set -x

cd /src/lg_ros_nodes/
source ./catkin/devel/setup.bash
./pack-debs master
