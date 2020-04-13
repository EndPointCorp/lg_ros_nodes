#!/bin/bash -ex

cd /src/lg_ros_nodes/catkin
. devel/setup.sh &&
cd /src/lg_ros_nodes
. ./scripts/docker_xvfb_add.sh
./scripts/test_runner.py
