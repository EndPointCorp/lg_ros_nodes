#!/bin/bash

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +
XSOCK=/tmp/.X11-unix

NV_DOCKER='sudo docker' nvidia-docker run \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --volume="${HOME}/.dbus:/root/.dbus" \
  --volume="/var/run/dbus/:/var/run/dbus/" \
  --volume="${DOCKER_DIR}/src:/home/lg/catkin_ws/src" \
  --volume="$XSOCK:$XSOCK:rw" \
  --volume="/tmp:/tmp/host_tmp" \
  --device="/dev/bus/usb:/dev/bus/usb" \
  --device="/dev/uinput:/dev/uinput" \
  "$@" \
  --interactive --tty --privileged lgros
