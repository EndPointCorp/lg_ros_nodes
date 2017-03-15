#!/bin/bash
if [ -d /home/galadmin/src/lg_ros_nodes_mounted/catkin ]; then
  echo "Detected local developer copy of ROS nodes mounted into container under /home/galadmin/src/lg_ros_nodes_mounted"
  echo "Here are all the files"
  APP_DIR=/home/galadmin/src/lg_ros_nodes_mounted
  ls -lha $APP_DIR
  DUID=$(stat -c '%u' $APP_DIR)
  DGID=$(stat -c '%g' $APP_DIR)
  if getent passwd galadmin > /dev/null 2>&1; then
      echo "Galadmin user exists - adjusting UID to match $APP_DIR perms"
      usermod -u $DUID galadmin
  else
      echo "Adding user galadmin with ID $DUID"
      useradd -u $DUID galadmin
  fi
  chown -R galadmin:galadmin $APP_DIR
  echo "Compiling project ($APP_DIR)"
  cd /home/galadmin/src/lg_ros_nodes_mounted/catkin
  catkin_make
else
  APP_DIR=/home/galadmin/src/lg_ros_nodes
  echo "Running image built-in (COPY) version of the APP - running on galadmin(uid=1000)"
fi
