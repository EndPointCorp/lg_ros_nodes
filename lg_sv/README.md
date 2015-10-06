lg\_sv
---------

ROS software for running a streetview/panoview client/server.

## Hardware requirements

* accelerated graphics cards
* decent CPU

## Software requirements

* google chrome available in PATH

## Nodes

### server

##### Overview

Creates rostopics to communicate/interact with the sv clients. Handles the spacenav input and publishes the changes to POV and triggers panoid changes.

##### Parameters

* `~tilt_max` [float] - Maximum value in degrees which the SV client can tilt. Default: `80`
* `~tilt_min` [float] - Minimum value in degress which the SV client can tilt. Default: `-80`
* `~nav_sensitivity` [float] - Value to adjust space nav angular changes by. Default: `1.0`
* `~space_nav_interval` [float] - Messages from the space nav are adjusted in relation to their frequency when updating the POV. An increase in frequency will reduce the impact of each message on the POV.  Default: `0.1`
* `~server_type` [string] - The type of server, `streetview` or `panoviewer`. Default: `streetview`
* `~x_threshold` [float] - The value that sets off a pano change when pushing
  forward or backward. Default: `0.5`
* `~nearby_class` [string] - The class desired for finding nearby panos, so far
  `NearbyPanos` and `NearbyStreetviewPanos` are the only supported types.
  Default: `NearbyStreetviewPanos`

##### Published Topics

* `/<server_type>/panoid` [string] - The current sv panoid
* `/<server_type>/pov` [Quaternion] - The current pov of the sv instance

##### Subscribed Topics

* `/<server_type>/location` [pose2d] - Lookup a panoid for the location and publish it if different than the current panoid.
* `/<server_type>/metadata` [string] - JSON representation of the sv metadata collected by the clients.  Save the list of links and their headings.  On move.forward compare current heading with list of links and find the link with the closest heading and update panoid.
* `/<server_type>/panoid` [string] - Set the servers self.panoid
* `/<server_type>/state` [ApplicationState] - The state of the sv application
* `/<server_type>/pov` [Quaternion] - Update the server's self.pov
* `/spacenav/twist` [Twist] - Update the POV according to the angular values sent from the nav (adjust according to msg frequency). Count consecutive forward or backward movements and when threshold is met update the panoid.

### client

##### Overview

Uses the sv launcher to start a browser with a sv instance. Add event listeners for POV, panoid msgs and links changed to the SV instance.  When changes come from POV and panoid topics via rosbridge emit a signal and let the sv instance update itself with the js api. The panoid of the current metadata doesn't match the actual current panoid, emit a signal to collect the metadata and publish it.

##### Parameters

* `~server_type` [string] - The type of server, `streetview` or `panoviewer`. Default: `streetview`
* `~url` [string] - The url of the webbapp. Default: `http://localhost:8008/lg_sv/webapps/client/index.html`
* `~fov` [string] - The field of view, how zoomed out we are from the images. Default: `28.125`
* `~pitch_offset` [float] - Pitch offset. Default: `0`
* `~show_links` [boolean] - Whether or not to show the arrows that point to the direction of nearby panos. Only really useful on the center pano. Default: `False`
* `~yaw_offset` [float] - The number of screens over from the center screen. `left_one` would be `-1`, `right_two` would be `2`, etc. Default: `0`
* `~leader` [boolean] - Used on one screen (panoviewer only) to determine the master for video syncing. Default: `false`
* `~tilt` [boolean] - Whether or not (streetview only) tilt will be handled. Default: `false`

##### Published Topics

* `/<server_type>/pov` [Quaternion] - Update the current POV
* `/<server_type>/metadata` [String] - Json of the metadata that has the current panoid and adjacent panoids and their heading.

##### Subscribed Topics

* `/<server_type>/pov` [Quaternion] - Update sv instance's POV
* `/<server_type>/panoid` [String] - Update sv instances current pano with a new panoid to swtihc to.
