Liquid Galaxy ROS Nodes
=======================

A ROS software stack for running Liquid Galaxy applications.

### Packages

#### lg\_earth

Earth client and support nodes.

Quick Start
-----------

Let's assume that you are using Ubuntu 14.04 and have Google Earth client, `ros-indigo-ros-base`, installed, and your `rosdep` updated.

* <https://dl.google.com/earth/client/current/google-earth-stable_current_i386.deb>
* <http://wiki.ros.org/indigo/Installation/Ubuntu>

Also, you'll need to patch Earth for the homedir fix as described in the lg\_earth README, otherwise its configuration will be unmanaged.

Additionally, you'll need to set up permissions for the special `/dev/uinput` file for the SpaceNav emulator to work. To make it persistent, write this udev rule to `/etc/udev/rules.d/42-uinput.rules`:

    SUBSYSTEM=="misc", KERNEL=="uinput", GROUP="plugdev", MODE:="0660"

This will fix uinput permissions at boot. You can also manually set permissions for the current session.

    $ sudo chown root:plugdev /dev/uinput ; sudo chmod 0660 /dev/uinput

Now then.

First, clone the repo (you can replace ~/src if you want).

    $ cd ~/src
    $ git clone git@github.com:EndPointCorp/lg_ros_nodes.git

Then run the init script.

    $ cd ~/src/lg_ros_nodes/
    $ ./scripts/init_workspace

It will warn you that appctl,  interactivespaces_msgs & lg_cms_director aren't there, and since the ros nodes depend on those, you should include them

    $ cd ~/src
    $ git clone git@github.com:EndPointCorp/portal-ros.git
    $ git clone git@github.com:EndPointCorp/ros_cms.git

Then re-run the init script with arguments to direct the script to those new repos

    $ cd ~/src/lg_ros_nodes
    $ ./scripts/init_workspace --appctl ~/src/portal-ros/catkin/src/appctl --interactive ~/src/ros_cms/director/src/interactivespaces_msgs --director ~/src/ros_cms/director/src/lg_cms_director

Install system dependencies with `rosdep`.

    $ cd ~/src/lg_ros_nodes/catkin
    $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

Build the project.

    $ cd ~/src/lg_ros_nodes/catkin
    $ catkin_make

As new ros nodes are added to this git repo, re-run `./scripts/init_workspace` and the `rosdep install` command.

Run the development roslaunch. You'll need to specify your local broadcast address, which can be found with `ifconfig`. Replace `1.2.3.255` with that address.

    $ source ~/src/lg_ros_nodes/catkin/devel/setup.bash
    $ roslaunch lg_common dev.launch broadcast_addr:=1.2.3.255

It may take a few seconds for Earth to start up. Use Ctrl+C to shut down.

If Earth isn't syncing, make sure that your firewall isn't blocking broadcast datagrams. If you're using ufw:

    $ sudo ufw allow to 1.2.3.255

Where 1.2.3.255 is your broadcast address.

## Development

LINT is configured, run `pep8` in the root of this repo to check Python
and use `catkin_lint` to check for errors in `package.xml` and
`CMakeLists.txt`.

## Making new release

- To make new release you need to:

```shell
$ catkin_generate_changelog
```

- Then edit all your `.rst` changelogs - remove unwanted or bogus messages
and make them look pretty. Use `catkin_generate_changelog --all` to
create `CHANGELOG.rst` for a new package.

- Once that's done, prepare release:

```shell
$ catkin_prepare_release
```
