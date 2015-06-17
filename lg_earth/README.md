lg\_earth
---------

ROS software for running and interfacing with the Google Earth desktop client.

### Nodes

#### client

Runs the Earth client.

A hack is required to move some of the configuration files out of the user's home directory. See the `earth_homedir_fix` script. (yet to be copied from live image)

##### Parameters

* `hide_gui` [bool] - Run headless, don't create a window. ViewSync and SpaceNav will still work. Default: `false`
* `viewsync_send` [bool] - Send viewsync datagrams. Default: `false`
* `viewsync_recv` [bool] - Receive viewsync datagrams. Default: `false`
* `viewsync_hostname` [string] - Hostname/IP for viewsync communication. Default: `10.42.42.255`
* `viewsync_port` [int] - Port for viewsync communication. Default: `42000`
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
* `mem_cache_size` [int] - Size of the memory cache in MB. Default: `32`
* `disk_cache_size` [int] - Size of the disk cache in MB. Default: `512`
* `show_state_borders` [bool] - Show state/province borders. Default: `false`
* `show_country_borders` [bool] - Show country borders. Default: `false`
* `show_state_labels` [bool] - Show state/province labels. Default: `false`
* `show_country_labels` [bool] - Show country labels. Default: `false`
* `show_city_labels` [bool] - Show city labels. Default: `false`
* `show_water_labels` [bool] - Show water body labels. Default: `false`
* `show_gray_buildings` [bool] - Show gray (untextured) 3D buildings. Default: `false`
* `show_buildings` [bool] - Show photorealistic (textured) 3D buildings. Default: `true`
* `show_trees` [bool] - Show 3D trees. Default: `true`
* `kml_sync_base` [string] - URL path to KML sync location. Default: `None`
* `kml_sync_slug` [string] - Identifier for KML sync. Default: `default`
* `default_view` [string] - KML AbstractView for starting location. Default: `<LookAt><longitude>-122.4661297737901</longitude><latitude>37.71903477888115</latitude><altitude>0</altitude><heading>42.60360249388481</heading><tilt>66.02791701475958</tilt><range>36611.51655091633</range><gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>`

#### viewsync\_sniffer

Intercepts Earth viewsync datagrams, publishes the `Pose`, and re-transmits the datagram. Best if run on the same host as the Earth ViewSync master client.

##### Parameters

* `listen_host` [string] - Host to bind listening socket to. Default: `127.0.0.1`
* `listen_port` [int] - Port to bind listening socket to. Default: `42000`
* `repeat_host` [string] - Host to repeat datagrams to. Default: `127.0.0.255`
* `repeat_port` [int] - Port to repeat datagrams to. Default: `42000`

##### Published Topics

* `/earth/pose` [geometry\_msgs/Pose] - The current Earth view in degrees and meters ASL.

