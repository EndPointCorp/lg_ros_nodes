#!/bin/bash
# Script used to syncronize catkin workspace between headnode and dispnodes

if [ -d .git ]; then
  echo "I'm going to sync catkin/src to dispnodes:catkin_ws/src/ and run catkin_make"
  echo "press enter to go"
  read
else
  echo "Not in a git repository - exiting"
  exit 1
fi

if [ -d catkin/src ] ; then
  echo "..."
else
  echo "There's no catkin/src - have you initialized your workspace?"
  exit 1
fi

echo 'installing libudev-dev'
if [[ ${SKIP_APT} != "" ]]; then
  echo "Skipping APT update + install"
else
  lg-sudo 'sudo apt-get update -q && sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good python-gst-1.0 libudev-dev gstreamer1.0-x ros-indigo-web-video-server -q -y'
fi

lg-sync --really-sync catkin/src/ /home/lg/catkin_ws/src/
lg-run-bg 'cd catkin_ws ; \
           source /opt/ros/indigo/setup.bash ; \
           rosdep install --from-paths src --ignore-src --rosdistro indigo -y;
           catkin_make install;\
           find /home/lg/catkin_ws/src/ -iname "*.pyc" -delete '

echo 'linking extensions to /opt/google/chrome/extensions/'
lg-sudo-bg 'sudo mkdir -p /opt/google/chrome/extensions/; sudo ln -sf /home/lg/catkin_ws/src/lg_common/src/lg_common/extensions/* /opt/google/chrome/extensions/'

echo "You need to relaunch to new version"
