#!/bin/bash

sess=docker
wn=roslaunch

tmux list-session 2>&1 | grep -q "^$sess" || tmux new-session -s $sess -d
tmux list-window -t $sess 2>&1 | grep -q ": $wn \[" || tmux new-window -t $sess -n $wn
tmux send-keys -t $sess:$wn "source /home/galadmin/src/lg_ros_nodes/catkin/devel/setup.bash" Enter
tmux send-keys -t $sess:$wn "roslaunch --screen lg_common/launch/dev.launch broadcast_addr:=127.0.0.255" Enter
tmux attach-session -t $sess
