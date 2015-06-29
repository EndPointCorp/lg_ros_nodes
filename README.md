Liquid Galaxy ROS Nodes
-----------------------

A ROS software stack for running Liquid Galaxy applications.

### Packages

#### lg\_earth

Earth client and support nodes.

Quick Start
===========

Let's assume that you are using Ubuntu 14.04 and have Google Earth client, `ros-indigo-ros-base`, installed, and your `rosdep` updated.

* <https://dl.google.com/earth/client/current/google-earth-stable_current_i386.deb>
* <http://wiki.ros.org/indigo/Installation/Ubuntu>

Also, you'll need to patch Earth for the homedir fix as described in the lg\_earth README, otherwise its configuration will be unmanaged.

Additionally, you'll need to set up permissions for the special `/dev/uinput` file for the SpaceNav emulator to work. To make it persistent, write this udev rule to `/etc/udev/rules.d/42-uinput.rules`:

    SUBSYSTEM=="misc", KERNEL=="uinput", GROUP="plugdev", MODE:="0660"

This will fix uinput permissions at boot. You can also manually set permissions for the current session.

    $ sudo chown root:plugdev /dev/uinput ; sudo chmod 0660 /dev/uinput

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

Install system dependencies with `rosdep`.

    $ cd ~/src/lg_ros_nodes/catkin
    $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

Build the project.

    $ cd ~/src/lg_ros_nodes/catkin
    $ catkin_make

Run the development roslaunch. You'll need to specify your local broadcast address, which can be found with `ifconfig`. Replace `1.2.3.255` with that address.

    $ source ~/src/lg_ros_nodes/catkin/devel/setup.bash broadcast_addr:=1.2.3.255
    $ roslaunch lg_common dev.launch

It may take a few seconds for Earth to start up. Use Ctrl+C to shut down.

If Earth isn't syncing, make sure that your firewall isn't blocking broadcast datagrams. If you're using ufw:

    $ sudo ufw allow to 1.2.3.255

Where 1.2.3.255 is your broadcast address.

Development
===========

LINT is configured, run `pep8` in the root of this repo to check.
