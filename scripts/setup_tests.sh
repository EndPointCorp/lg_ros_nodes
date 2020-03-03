#!/bin/bash -ex


cd $WORKSPACE
git clean -fdx
rm -rf "$WORKSPACE/catkin"
source /opt/ros/melodic/setup.bash

./scripts/init_workspace

#cd catkin/src
#git clone git@github.com:EndPointCorp/appctl.git -b master

#cd ..
catkin_make

cd $WORKSPACE

# loads the environment
source catkin/devel/setup.bash
