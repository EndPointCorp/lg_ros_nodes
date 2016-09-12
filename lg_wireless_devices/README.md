## lg\_wireless\_devices

Ros package, which allows to execute command by hitting wiress device buttons.

Wireless device can be connected to display or head node of a Liquid Galaxy system, commands will be executed on the headnode.

### ROS Nodes

- #### wireless\_watcher

  looks for `wireless_watcher.conf` which contains
  mapping between device events and commands and
  publish commands to `/command_handler` topic.

  ##### ROS Parameters
  * `udev_location` location of `rules` file, with mapping
  between commands names and `os.execute` code.
  Both file and url allowed.

- #### command\_handler

  Listens for commands from `/command_handler` topic,
  and runs commands from `*.rules` file.

  ##### ROS Parameters
  * `config_location` location of `config` file.
  Both file and url allowed.


### ROS Topics

* `/command_handler` of type `misc/Command`

  Topic for commands transfer. Commands are just names,
  actual shell commands are executed using
  `command_handler` `rules` file.


### ROS Messages

* `misc/Command`
