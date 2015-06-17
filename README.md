Liquid Galaxy ROS Nodes
-----------------------

A ROS software stack for running Liquid Galaxy applications.

### Packages

#### lg\_earth

Earth client and support nodes.

### Quick Start

First, clone the repo.

    $ cd ~/src
    $ git clone git@github.com:EndPointCorp/lg_ros_nodes.git

Create a catkin workspace for this stack and symlink it back to the code.

    $ mkdir -p ~/src/lg_ros_nodes/catkin/src
    $ cd ~/src/lg_ros_nodes/src
    $ catkin_init_workspace
    $ ln -snf ~/src/lg_ros_nodes/lg_earth

Now you'll need to symlink `appctl` from the Portal ROS repo and `interactivespaces_msgs`from the director repo.

    $ cd ~/src
    $ git clone git@github.com:EndPointCorp/portal-ros.git
    $ git clone git@github.com:EndPointCorp/ros_cms.git
    $ cd ~/src/lg_ros_nodes/catkin/src
    $ ln -snf ~/src/portal-ros/catkin/src/appctl
    $ ln -snf ~/src/ros_cms/director/src/interactivespaces_msgs

Build the project.

    $ cd ~/src/lg_ros_nodes/catkin
    $ catkin_make

Run a node.

    $ source ~/src/lg_ros_nodes/catkin/devel/setup.bash
    $ rosrun lg_earth client.py

