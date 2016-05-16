lg\_earth
---------

ROS software for running and interfacing with the Google Earth desktop client.

## Hardware requirements

* accelerated graphics supported by GE (every decent nvidia card)

## Software requirements

* Google Earth (see below)
* awesome window manager

### System setup

Let's assume that you are using Ubuntu 14.04 and have Google Earth client, `ros-indigo-ros-base`, installed, and your `rosdep` updated.

* <https://dl.google.com/earth/client/current/google-earth-stable_current_i386.deb>
* <http://wiki.ros.org/indigo/Installation/Ubuntu>

Also, you'll need to patch Earth for the homedir fix as described in the lg\_earth README, otherwise its configuration will be unmanaged.

Additionally, you'll need to set up permissions for the special `/dev/uinput` file for the SpaceNav emulator to work. To make it persistent, write this udev rule to `/etc/udev/rules.d/42-uinput.rules`:

    SUBSYSTEM=="misc", KERNEL=="uinput", GROUP="plugdev", MODE:="0660"

This will fix uinput permissions at boot. You can also manually set permissions for the current session.

    $ sudo chown root:plugdev /dev/uinput ; sudo chmod 0660 /dev/uinput

Run the development roslaunch. You'll need to specify your local broadcast address, which can be found with `ifconfig`. Replace `1.2.3.255` with that address.

    $ source ~/src/lg_ros_nodes/catkin/devel/setup.bash
    $ roslaunch lg_common dev.launch broadcast_addr:=1.2.3.255

It may take a few seconds for Earth to start up. Use Ctrl+C to shut down.

If Earth isn't syncing, make sure that your firewall isn't blocking broadcast datagrams. If you're using ufw:

    $ sudo ufw allow to 1.2.3.255

Where 1.2.3.255 is your broadcast address.

### Language Support

To set the language locale for Google Earth set the `LG_LANG` enviromental variable.

### Nodes

#### client

Runs the Earth client.

A hack is required to move some of the configuration files out of the user's home directory. Earth will read /etc/passwd to find the current user's home directory instead of respecting the HOME environment variable. We can work around this by changing the string "/etc/passwd" to a void path in the library that does this lookup.

A script to achieve this:

    #!/usr/bin/env bash
    LIB=/opt/google/earth/free/libfreebl3.so
    [ -e $LIB.orig ] || cp $LIB $LIB.orig
    sed -i "s_/etc/passwd_/not/anywhr_g" $LIB

Run with sudo.

##### Parameters

* `hide_gui` [bool] - Run headless, don't create a window. ViewSync and SpaceNav will still work. Default: `false`
* `viewsync_send` [bool] - Send viewsync datagrams. Default: `false`
* `viewsync_host` [string] - Hostname/IP for viewsync communication. Default:
  `10.42.42.255`
* `viewsync_port_new` [int] - Port for viewsync communication. Default: `42000`
* `query_file` [string] - Path to the query.txt file. Default: `/tmp/ge_queryfile`
* `horiz_fov` [float] - Horizontal field of view in degrees. Default: `65`
* `yaw_offset` [float] - Yaw view offset in degrees. Default: `0`
* `pitch_offset` [float] - Pitch view offset in degrees. Default: `0`
* `roll_offset` [float] - Roll view offset in degrees. Default: `0`
* `spacenav_device` [string] - Path to SpaceNav evdev node. Default: `None`
* `spacenav_gutter` [float] - SpaceNav gutter value. Default: `0.1`
* `spacenav_sensitivity` [float] - Overall SpaceNav sensitivity ratio. Default: `1.0`
* `spacenav_sensitivity_yaw` [float] - SpaceNav relative yaw sensitivity. Default: `0.0035`
* `spacenav_sensitivity_pitch` [float] - SpaceNav relative pitch sensitivity. Default: `0.01`
* `spacenav_sensitivity_roll` [float] - SpaceNav relative roll sensitivity. Default: `0.01`
* `spacenav_sensitivity_x` [float] - SpaceNav relative x translation sensitivity. Default: `0.25`
* `spacenav_sensitivity_y` [float] - SpaceNav relative y translation sensitivity. Default: `0.25`
* `spacenav_sensitivity_z` [float] - SpaceNav relative z translation sensitivity. Default: `0.025`
* `flyto_speed` [float] - Speed for flyTo queries. Default: `0.17`
* `show_compass` [bool] - Show compass navigator. Default: `false`
* `show_visualization` [bool] - Show SpaceNav/LEAP visualization. Default: `true`
* `use_3d_imagery` [bool] - Show new 3D imagery. Default: `true`
* `anisotropic_filtering` [int] - Anisotropic filtering level. Can be 0, 1, or 2. Default: `2`
* `high_quality_terrain` [bool] - Show high quality terrain. Default: `true`
* `texture_compression` [bool] - Use texture compression. Default: `true`
* `status_bar_visible` [bool] - Show the status bar at the bottom of the window. Default: `true`
* `mem_cache_size` [int] - Size of the memory cache in MB. Default: `64`
* `disk_cache_size` [int] - Size of the disk cache in MB. Default: `256`
* `show_state_borders` [bool] - Show state/province borders. Default: `false`
* `show_country_borders` [bool] - Show country borders. Default: `false`
* `show_state_labels` [bool] - Show state/province labels. Default: `false`
* `show_country_labels` [bool] - Show country labels. Default: `false`
* `show_city_labels` [bool] - Show city labels. Default: `false`
* `show_water_labels` [bool] - Show water body labels. Default: `false`
* `show_gray_buildings` [bool] - Show gray (untextured) 3D buildings. Default: `false`
* `show_buildings` [bool] - Show photorealistic (textured) 3D buildings. Default: `true`
* `show_trees` [bool] - Show 3D trees. Default: `true`
* `show_google_logo` [bool] - Show the Google Earth logo at the bottom of the window. Default: `true`
* `custom_configs` [string] - Url and filename for config files "<url>,filename;<url2>,filename2;..."
* `kml_sync_base` [string] - URL path to KML sync location. Default: `None`
* `kml_sync_slug` [string] - Identifier for KML sync. Default: `default`
* `default_view` [string] - KML AbstractView for starting location. Default: `<LookAt><longitude>-122.4661297737901</longitude><latitude>37.71903477888115</latitude><altitude>0</altitude><heading>42.60360249388481</heading><tilt>66.02791701475958</tilt><range>36611.51655091633</range><gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>`

#### viewsync\_relay

Intercepts Earth viewsync datagrams, publishes the `Pose`, and re-transmits the datagram. Best if run on the same host as the Earth ViewSync master client.

##### Parameters

* `listen_host` [string] - Host to bind listening socket to. Default: `127.0.0.1`
* `listen_port` [int] - Port to bind listening socket to. Default: `42000`
* `repeat_host` [string] - Host to repeat datagrams to. Default: `<broadcast>`
* `repeat_port` [int] - Port to repeat datagrams to. Default: `42000`

##### Published Topics

* `/earth/pose` [`geometry\_msgs/PoseStamped`] - The current Earth view in degrees and meters ASL.

Breakdown of a `PoseStamped`:

```
header:
  seq: [int] # sequence number? helpful for out of order packets maybe
  stamp:
    secs: int # seconds since epoch?
    nsecs: int # nanoseconds since seconds since epoch
  frame_id: string # no idea

# Now for the usefull stuff
pose:
  position:
    x: float # latitude
    y: float # longitude
    z: float # altitude
  orientation:
    x: float # tilt
    y: float # roll
    z: float # heading
    w: 0 # always 0
```

#### query

Listens on topics for queries to write to the Earth query file.

##### Parameters

* `~query_file` [string] - Path to the Earth query file. Default: `/tmp/ge_queryfile`
* `~queue_length` [int] - Number of queries to queue up. Default: `10`

##### Subscribed Topics

* `/earth/query/flyto_kml` [`std_msgs/String`] - A KML `AbstractView` to fly to.
* `/earth/query/flyto_pose_camera` [`geometry_msgs/Pose`] - A `Pose` expressed in latitude, longitude, degrees, and meters ASL. A `<Camera>` view will be generated with the absolute `Pose` values.
* `/earth/query/flyto_pose_lookat` [`geometry_msgs/Pose`] - A `Pose` expressed in latitude, longitude, degrees, and meters relative to sea floor. A `<LookAt>` view will be generated, with heading, tilt, and roll based on the `Pose` orientation.
* `/earth/query/search` [`std_msgs/String`] - Search string.
* `/earth/query/tour` [`std_msgs/String`] - Play a tour by its `id`. An empty string will `exittour`.
* `/earth/query/planet` [`std_msgs/String`] - Change planets.

#### planet\_changer

Content-triggered planet switching.  Changes planets based on the name of a selected presentation.  If the selected presentation does not match the Moon or Mars groups, Earth is selected.

##### Parameters

* `~moon_presentations` [string] - Semicolon-separated list of presentations to run on the Moon.  Default: `Moon`
* `~mars_presentations` [string] - Semicolon-separated list of presentations to run on the Mars.  Default: `Mars`

##### Subscribed Topics

* `/director/presentation` [`interactivespaces_msgs/GenericMessage`]

##### Published Topics

* `/earth/query/planet` [`std_msgs/String`]
