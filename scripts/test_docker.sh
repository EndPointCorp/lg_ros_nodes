#!/bin/bash -x

function setup_files() {
  ORIG_UMASK=`umask`
  umask 0022
  # make a destination for the files
  mkdir -p docker_nodes/
  chmod 0755 docker_nodes
  # copy all nodes (following links) into the proper directory
  cp -faL catkin/src/* docker_nodes/ || true
  umask "${ORIG_UMASK}"
}

function initialize() {
  # generate a unique image/container id
  UUID=`mktemp -u XXXXXXXXXXXXXXXXXXXXXXXX`
  UUID="${UUID,,}"
  DOCKER_NAME="$(basename $(pwd))-test-${UUID}"
}

# builds the docker image, naming it <project-name>-test depending
# on the dirname of the project, e.g. lg_ros_nodes-test
function build_docker() {
  echo building ${DOCKER_NAME}
  docker build --pull -t ${DOCKER_NAME} .
}

function start_xvfb () {
    #/etc/init.d/xvfb start
    #export DISPLAY=:1.0

    echo "Installing Xvfb"

    sudo apt-get install -q -y x11-apps xvfb

    msg="Started Xvfb"

    xinerama="+xinerama"
    if [ "$NO_XINERAMA" = "true" ]; then
        xinerama=""
        msg="$msg, xinerama disabled"
    fi
    randr="+extension RANDR"
    if [ "$NO_RANDR" = "true" ]; then
        randr=""
        msg="$msg, randr disabled"
    fi

    Xvfb :1 $randr $xinerama -nolock \
        -nocursor -screen 0 1920x1080x24 \
        &> /tmp/xvfb_start.log &

    export DISPLAY=:1

    msg="$msg, DISPLAY=$DISPLAY"
    echo $msg
}

# runs tests and returns the return value
function run_tests() {
  echo running ${DOCKER_NAME}
  docker run \
    --name ${DOCKER_NAME} \
    --rm \
    --volume="$(pwd)/docker_nodes:/docker_nodes:ro" \
    ${DOCKER_NAME}
  RETCODE=$?
}

# cleans up all test artifacts
function cleanup() {
  echo cleaning up

  if docker ps -a | grep ${DOCKER_NAME} >/dev/null; then
    docker rm -f ${DOCKER_NAME}
  fi

  if docker images | grep ${DOCKER_NAME} >/dev/null; then
    docker rmi ${DOCKER_NAME}
  fi

  sleep 0.5

  dangling="$(docker images --filter "dangling=true" -q --no-trunc 2>/dev/null)"
  if [ -n "$dangling" ]; then
    docker rmi "$dangling"
  fi

  exit $RETCODE
}

set -e

if [ "$NO_XVFB" != "true" ]; then
    start_xvfb
else
    xclock &
    XC_PID=$(pgrep xclock)
    if [ "x$XC_PID" == "x" ]; then
        echo "Couldn't connect to Xorg server"
        if [ ! -f /tmp/.X11/X0 ]; then
            echo "Xorg socket (/tmp/.X11/X0) not found."
        fi
    fi
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd ${DIR}/..
DOCKER_NAME=
RETCODE=1

initialize
setup_files

set +e
trap cleanup EXIT

build_docker || exit 1
run_tests
