#!/bin/bash -x

function setup_files() {
  ORIG_UMASK="$(umask)"
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
  UUID="$( base64 /dev/urandom | tr -d '/+[A-Z]' | dd bs=16 count=1 2>/dev/null )"
  DOCKER_NAME="$(basename $(pwd))-test-${UUID}"
}

# builds the docker image, naming it <project-name>-test depending
# on the dirname of the project, e.g. lg_ros_nodes-test
function build_docker() {
  echo building "${DOCKER_NAME}"
  docker build --pull -t "${DOCKER_NAME}" .
}


# runs tests and returns the return value
function run_tests() {
  echo running "${DOCKER_NAME}" in "$(pwd)"
  docker run \
    --name "${DOCKER_NAME}" \
    -u 0 \
    --volume="$(pwd)/docker_nodes:/docker_nodes:ro" \
    --rm \
    --env="DISPLAY=:1" \
    "${DOCKER_NAME}" \
    /bin/bash
     # -c '
     #  cd ${PROJECT_ROOT}/catkin && \
     #  . devel/setup.sh && \
     #  cd ${PROJECT_ROOT} && \
     #  ./scripts/docker_xvfb_add.sh && \
     #  ./scripts/test_runner.py
     #'
  RETCODE=$?
}

#ROS_PYTHON_VERSION=2 catkin build; ROS_PYTHON_VERSION=3 catkin build; catkin run_tests; catkin_test_results

# cleans up all test artifacts
function cleanup() {
  echo cleaning up

  if docker ps -a | grep "${DOCKER_NAME}" >/dev/null; then
    docker rm -f "${DOCKER_NAME}"
  fi

  if docker images | grep "${DOCKER_NAME}" >/dev/null; then
    docker rmi "${DOCKER_NAME}"
  fi

  exit "$RETCODE"
}

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."
DOCKER_NAME=
RETCODE=1

initialize
setup_files

set +e
#trap cleanup EXIT

build_docker || exit 1
run_tests
