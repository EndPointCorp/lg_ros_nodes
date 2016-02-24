#!/bin/sh

version_gt() { test "$(echo "$@" | tr " " "\n" | sort -V | tail -n 1)" = "$1"; }
docker_version=$(sudo docker version | grep 'Client version' | awk '{split($0,a,":"); print a[2]}' | tr -d ' ')
# Docker 1.3.0 or later is required for --device
if ! version_gt "${docker_version}" "1.2.0"; then
	echo "Docker version 1.3.0 or greater is required"
	exit 1
fi

if test $# -lt 1; then
	# Get the latest opengl-nvidia build
	# and start with an interactive terminal enabled
	args="-i -t $(sudo docker images | grep ^lg_ros_nodes | head -n 1 | awk '{ print $1":"$2 }')"
else
        # Use this script with derived images, and pass your 'docker run' args
	args="$@"
fi

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
sudo touch $XAUTH
sudo xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | sudo xauth -f $XAUTH nmerge -

sudo docker run \
	-v $XSOCK:$XSOCK:rw \
	-v $XAUTH:$XAUTH:rw \
        --device=/dev/dri/card0:/dev/dri/card0 \
	-e DISPLAY=$DISPLAY \
	-e XAUTHORITY=$XAUTH \
	--net=host \
	-v /tmp:/tmp/host_tmp \
	$args
