Liquid Galaxy ROS Nodes
=======================

A ROS software stack for running Liquid Galaxy applications.

First, clone the repos (you can replace ~/src if you want).

    $ cd ~/src
    $ git clone git@github.com:EndPointCorp/lg_ros_nodes.git
    $ git clone git@github.com:EndPointCorp/appctl.git

Then run the init script.

    $ cd ~/src/lg_ros_nodes/
    $ ./scripts/init_workspace

Then re-run the init script with arguments to direct the script to those new repos

    $ cd ~/src/lg_ros_nodes
    $ ./scripts/init_workspace --appctl ~/src/appctl

Install system dependencies with `rosdep`.

    $ cd ~/src/lg_ros_nodes/catkin
    $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

Build the project.

    $ cd ~/src/lg_ros_nodes/catkin
    $ catkin_make

As new ros nodes are added to this git repo, re-run `./scripts/init_workspace` and the `rosdep install` command.

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
