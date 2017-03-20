#!/bin/bash
set -e

if [ -f /home/galadmin/src/lg_ros_nodes_mounted/catkin/devel/setup.bash ]; then
  echo "LG version found under /home/galadmin/src/lg_ros_nodes_mounted/catkin/devel/setup.bash"
  profile_file="/home/galadmin/src/lg_ros_nodes_mounted/catkin/devel/setup.bash"
else
  profile_file="/opt/ros/indigo/setup.bash"
fi
echo "Sourcing profile from $profile_file"

source $profile_file

exec "$@"
