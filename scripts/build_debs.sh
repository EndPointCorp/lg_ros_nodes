#!/bin/bash -x

function initialize() {
  # generate a unique image/container id
  UUID="$( base64 /dev/urandom | tr -d '/+[A-Z]' | dd bs=16 count=1 2>/dev/null )"
  DOCKER_NAME="${PROJECT_NAME}-build-${UUID}"
}

function build_base() {
  echo "building base image for ${ROS_DISTRO} on ${OS_VERSION}"
  docker build -f lg_ros_nodes_base/Dockerfile \
    --pull --rm=true --force-rm \
    --build-arg build_os_version="${OS_VERSION}" \
    --build-arg build_ros_distro="${ROS_DISTRO}" \
    -t "lg_ros_nodes_base:${ROS_DISTRO}" .
}

# builds the docker image, naming it <project-name>-test depending
# on the dirname of the project, e.g. lg_ros_nodes-test
function build_docker() {
  echo "building ${DOCKER_NAME} for ${ROS_DISTRO} on ${OS_VERSION}"
  docker build \
    --rm=true --force-rm \
    --build-arg build_os_version="${OS_VERSION}" \
    --build-arg build_ros_distro="${ROS_DISTRO}" \
    -t "${DOCKER_NAME}" .
}

# runs tests and returns the return value
function run_tests() {
  echo "testing ${DOCKER_NAME} for ${ROS_DISTRO} on ${OS_VERSION}"
  docker run \
    --name "${DOCKER_NAME}" \
    --rm \
    "${DOCKER_NAME}"
}

function build_debs() {
  echo "buildings debs ${DOCKER_NAME} for ${ROS_DISTRO} on ${OS_VERSION}"
  mkdir -p .build
  docker run \
    --name "${DOCKER_NAME}" \
    --rm \
    -v "$(pwd)/.build:/output:rw" \
    "${DOCKER_NAME}" bash -ec "
cd /home/galadmin/src/lg_ros_nodes
source /opt/ros/${ROS_DISTRO}/setup.bash
find . -name package.xml -print0 | xargs -0 bash -ec 'for filename; do lg-ros-build \$(dirname \$filename) --ros_distro=${ROS_DISTRO} --os_version=${OS_VERSION}; done' bash
mv *.deb /output/
"
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

build_base || exit 1
build_docker || exit 1
run_tests || exit 1
build_debs
