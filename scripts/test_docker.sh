#!/bin/bash -x

function initialize() {
  # generate a unique image/container id
  UUID="$( base64 /dev/urandom | tr -d '/+[A-Z]' | dd bs=16 count=1 2>/dev/null )"
  DOCKER_NAME="${PROJECT_NAME}-test-${UUID}"
}

# builds the docker image, naming it <project-name>-test depending
# on the dirname of the project, e.g. lg_ros_nodes-test
function build_docker() {
  echo "building ${DOCKER_NAME}"
  docker build \
    --rm=true --force-rm \
    --build-arg build_os_version="${OS_VERSION}" \
    --build-arg build_ros_distro="${ROS_DISTRO}" \
    -t "${DOCKER_NAME}" .
}


# runs tests and returns the return value
function run_tests() {
  echo "running ${DOCKER_NAME} in $(pwd)"
  docker run \
    --name "${DOCKER_NAME}" \
    --rm \
    "${DOCKER_NAME}"
  RETCODE=$?
}

# cleans up all test artifacts
function cleanup() {
  echo "cleaning up"

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
PROJECT_NAME=$(basename "$(pwd)")
DOCKER_NAME=
RETCODE=1
OS_VERSION="${BUILD_OS_VERSION:-trusty}"
ROS_DISTRO="${BUILD_ROS_DISTRO:-indigo}"

initialize

set +e
trap cleanup EXIT

build_docker || exit 1
run_tests
