# Liquid Galaxy

![liquidgalaxy](liquidgalaxy.jpg "lg image")

This repo contains ROS software for running Liquid Galaxy applications.

It allows for displaying engaging content in an immersive environment.
Currently it supports:
- [Google Earth](https://earth.google.com/) with KML support
- [Streetview](https://www.google.com/maps/streetview/)
- [mplayer](https://www.mplayerhq.hu/)
- [Chrome](https://www.google.com/chrome)
and...
- every asset that is supported by above apps

# General requirements

NOTE: all requirements specific to ros_nodes are in their respective
README.md files

- [Ubuntu 14.04 LTS](http://releases.ubuntu.com/14.04/)
- [ros-indigo](http://wiki.ros.org/indigo)
- [awesome window manager](http://awesome.naquadah.org/) on the top of
  [Xorg](https://wiki.archlinux.org/index.php/Xorg) for automatic window positioning
- only Nvidia hardware was tested but it should be running with whatever
  decent accelerated graphics card
- possibility to modify [udev
  configuration](https://en.wikipedia.org/wiki/Udev)
- [spacenavigator](http://www.3dconnexion.com/products/spacemouse/spacenavigator.html)
- industry standard touchscreen like [elo
  touch 2201L](http://www.elotouch.com/products/lcds/2201L/) is also a good input device
- it's also good to manage your stack with [chef](https://www.chef.io/chef/)

## Making it running

First, clone the repos (you can replace ~/src if you want).

```bash
$ cd ~/src
$ git clone git://github.com/EndPointCorp/lg_ros_nodes.git
$ git clone git://github.com/EndPointCorp/appctl.git
```

Then run the init script.

```bash
$ cd ~/src/lg_ros_nodes/
$ ./scripts/init_workspace
```

Then re-run the init script with arguments to direct the script to those new repos

```bash
$ cd ~/src/lg_ros_nodes
$ ./scripts/init_workspace --appctl ~/src/appctl/appctl
```

Install system dependencies with `rosdep`.

```bash
$ cd ~/src/lg_ros_nodes/catkin
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y
```

Build the project.

```
$ cd ~/src/lg_ros_nodes/catkin
$ catkin_make
```

Run Google Earth, Streetview and Panoviewer sample .launch file:

```bash
roslaunch --screen lg_common/launch/dev.launch
```

NOTE: As new ros nodes are added to this git repo, re-run `./scripts/init_workspace` and the `rosdep install` command.

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
