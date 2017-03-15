#!/bin/sh
set -e

/prepare.sh

if [ -d /home/galadmin/src/lg_ros_nodes_mounted/catkin ]; then
  export APP_DIR="/home/galadmin/src/lg_ros_nodes_mounted"
  echo "Detected local developer copy of ROS nodes mounted into container $APP_DIR"
else
  export APP_DIR="/home/galadmin/src/lg_ros_nodes"
  echo "Detected image bundled ROS nodes $APP_DIR"
fi
echo "---- LAUNCHING ROS FROM $APP_DIR ----"

roslaunch_file="$APP_DIR/roslaunch/$ROSLAUNCH_TARGET"

if [ -f $roslaunch_file ]; then
  echo "Launching $roslaunch_file"
else
  echo "Roslaunch file ($roslaunch_file) not found - exiting"
  exit 1
fi
/etc/init.d/nginx restart

/ros_entrypoint.sh roslaunch --screen $roslaunch_file
