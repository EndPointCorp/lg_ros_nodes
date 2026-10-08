#!/usr/bin/env bash
#
# Run the offline test suite against your working tree, on a machine with no
# ROS toolchain.
#
#   scripts/test_local.sh                       # whole offline suite
#   scripts/test_local.sh -x -k handle_scene    # extra args go to pytest
#   LG_TEST_IMAGE=endpoint/lg_ros_nonfree:1.2.36-durdel-2 scripts/test_local.sh
#
# The tests run inside the lg_ros_nonfree image, which carries the visionport
# runtime the display nodes actually use. scripts/test_local_run.py stages the
# working tree into the image's install-space layout, transpiles it to the
# vpros dialect, and runs every test registered with catkin_add_nosetests.
#
# This covers the offline suite only. The rostest-driven online tests still
# need a real ROS toolchain (see scripts/test_docker.sh and the Dockerfile).
#
# A throwaway MQTT broker runs alongside on a private docker network: vpros
# will not hand out parameters or publishers without a node, and a node needs
# a broker. Nothing here touches the production broker on lg-head.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LG_TEST_IMAGE:-lg-head:5000/lg_ros_nonfree:lg}"
BROKER_IMAGE="${LG_TEST_BROKER_IMAGE:-eclipse-mosquitto:2}"

ID="lgtest-$$"
NETWORK="${ID}-net"
BROKER="${ID}-mq"

cleanup() {
    docker rm -f "${BROKER}" >/dev/null 2>&1 || true
    docker network rm "${NETWORK}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker network create "${NETWORK}" >/dev/null

docker run -d --rm --name "${BROKER}" --network "${NETWORK}" "${BROKER_IMAGE}" \
    sh -c 'printf "listener 1883 0.0.0.0\nallow_anonymous true\n" > /mosquitto.conf
           exec mosquitto -c /mosquitto.conf' >/dev/null

# Give mosquitto a moment; vpros gives up rather than retrying a refused
# connect. Probe with mosquitto_pub: busybox nc has no -z, so it reports
# failure however healthy the broker is.
for _ in $(seq 40); do
    if docker exec "${BROKER}" mosquitto_pub -h localhost -t _probe -m up >/dev/null 2>&1; then
        break
    fi
    sleep 0.25
done

# ManagedBrowser unconditionally asks lg-head for API headers on construction,
# with a five second timeout. Point the name at the loopback so the connection
# is refused at once instead of hanging every browser the suite builds. Tests
# have no business reaching the network.
docker run --rm -u 0 \
    --network "${NETWORK}" \
    --add-host "lg-head:127.0.0.1" \
    -e "MQ_HOST=${BROKER}" \
    -e MQ_PORT=1883 \
    --volume "${REPO}:/src:ro" \
    --entrypoint python3 \
    "${IMAGE}" /src/scripts/test_local_run.py "$@"
