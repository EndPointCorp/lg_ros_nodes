#!/usr/bin/env bash

set -e
set -x


cp -r /root/ssh /root/.ssh
chown -R root:root /root/.ssh
chmod 0600 /root/.ssh/id_rsa
eval $(ssh-agent -s)

ssh -o StrictHostKeychecking=no aptly@${APTLY_SERVER} mkdir -p incoming/lg_ros_nodes/origin/master/
cd /src/lg_ros_nodes/catkin/
scp -o StrictHostKeychecking=no ./debs/*.deb aptly@${APTLY_SERVER}:incoming/lg-ros/origin/master/
ssh -o StrictHostKeychecking=no  aptly@${APTLY_SERVER} bash /home/aptly/bin/publish-incoming.sh --project lg-ros --branch origin/master --rosrel "noetic" --distro "focal"
ssh -o StrictHostKeychecking=no aptly@${APTLY_SERVER} bash /home/aptly/bin/publish-incoming-separate-repos.sh --project lg-ros --branch origin/master --rosrel "noetic" --distro "focal"
