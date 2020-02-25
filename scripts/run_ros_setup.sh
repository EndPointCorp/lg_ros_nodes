#!/bin/bash -ex


pwd
/var/jenkins_home/workspace/lg_ros_node_master/scripts/init_workspace

git clean -fdx

mkdir -p catkin/
cd catkin
rm -rf devel/ build/
mkdir -p src

cd src
rm -fr appctl
git clone git@github.com:EndpointCorp/appctl.git -b master

cd ..
catkin_make
cd ..
source catkin/devel/setup.bash
