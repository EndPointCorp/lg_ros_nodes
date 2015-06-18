Liquid Galaxy ROS Nodes
-----------------------

A ROS software stack for running Liquid Galaxy applications.

### Packages

#### lg\_earth

Earth client and support nodes.

### Quick Start

Let's assume that you are using Ubuntu 14.04 and have Google Earth client, ros-indigo-ros-base, and xdotool installed.

* <https://dl.google.com/earth/client/current/google-earth-stable_current_i386.deb>
* <http://wiki.ros.org/indigo/Installation/Ubuntu>
* <http://packages.ubuntu.com/trusty/xdotool>

Also, you'll need to patch Earth for the homedir fix as described in the lg\_earth README, otherwise its configuration will be unmanaged.

Now then.

First, clone the repo.

    $ cd ~/src
    $ git clone git@github.com:EndPointCorp/lg_ros_nodes.git

Create a catkin workspace for this stack and symlink it back to the code.

    $ mkdir -p ~/src/lg_ros_nodes/catkin/src
    $ cd ~/src/lg_ros_nodes/catkin/src
    $ catkin_init_workspace
    $ ln -snf ~/src/lg_ros_nodes/lg_common
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

