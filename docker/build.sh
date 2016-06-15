#!/bin/bash

# Get your current host nvidia driver version, e.g. 340.24
nvidia_version=$(cat /proc/driver/nvidia/version | head -n 1 | awk '{ print $8 }')

# We must use the same driver in the image as on the host
if test ! -f nvidia-driver.run; then
  nvidia_driver_uri=http://us.download.nvidia.com/XFree86/Linux-x86_64/${nvidia_version}/NVIDIA-Linux-x86_64-${nvidia_version}.run
  wget -O nvidia-driver.run $nvidia_driver_uri
fi

BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $BASE/..
rsync -azRL --exclude docker/ --exclude .git/ ./ ./docker/workspace_copy
mkdir ./docker/workspace_copy/.git

cd $BASE
sudo docker build -t lg_ros_nodes:${nvidia_version} .
sudo docker tag -f lg_ros_nodes:${nvidia_version} lg_ros_nodes:latest
