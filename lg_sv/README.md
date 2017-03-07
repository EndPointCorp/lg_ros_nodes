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
* `~zoom_min` [float] - Magic number marking the most the panoviewer's POV will
  zoom in, NOTE: to remove zoom set `zoom_min` equal to `zoom_max`. Default:
  `10`
* `~zoom_max` [float] - Magic number marking the most the panoviewer's POV will
  zoom out, NOTE: to remove zoom set `zoom_min` equal to `zoom_max`. Default:
  `30`
* `~nav_sensitivity` [float] - Value to adjust space nav angular changes by. Default: `1.0`
* `~space_nav_interval` [float] - Messages from the space nav are adjusted in relation to their frequency when updating the POV. An increase in frequency will reduce the impact of each message on the POV.  Default: `0.1`
* `~server_type` [string] - The type of server, `streetview` or `panoviewer`. Default: `streetview`
* `~x_threshold` [float] - The value that sets off a pano change when pushing
  forward or backward. NOTE: this should be ~30% less than the max value emitted
  on the spacenav when pushing forward / backward. For regular spacenav the max
  value is 0.68, but for fullscale spacenavs this value is `255`. Default: `0.5`
  (again if you use fullscale you should set this to `180` or so)
* `~nearby_class` [string] - The class desired for finding nearby panos, so far
  `NearbyPanos` and `NearbyStreetviewPanos` are the only supported types.
  Default: `NearbyStreetviewPanos`
* `~inverted` [string] - "true" or "false" determining whether or not the pov
  will be inverted. The `lg_sv_nonfree` viewer needs to have an inverted server,
  nothing else does.
* `~tick_rate` [float] - Frequency of pov updates from the server.  Default: `180.0`

##### Published Topics

* `/<server_type>/panoid` [String] - The current sv panoid.
* `/<server_type>/pov` [Quaternion] - The current pov of the sv instance.
* `/<server_type>/location` [Pose2D] - The current lat/lng of streetview. Not
  currently used.
* `/<server_type>/metadata` [String] - Publishes own metadata. This is used when
  someone publishes on `/<server_type>/raw_metadata`, the raw metadata will be
  converted to the proper format of metadata and published on this topic.
* `/<server_type>/state` [ApplicationState] - Sets own state depending on asset
  types in director messages. e.g. if a streetview asset is published by the
  director, the streetview app will set its state to VISIBLE, otherwise HIDDEN.

##### Subscribed Topics

* `/<server_type>/location` [pose2d] - Lookup a panoid for the location (lat/lng) and publish it  on `/<server_type>/panoid` if different than the current panoid.
* `/<server_type>/metadata` [string] - JSON representation of the sv metadata collected by the clients.  Save the list of links and their headings.
* `/<server_type>/panoid` [string] - Sets the server's self.panoid
* `/<server_type>/state` [ApplicationState] - Sets he state of the sv
  application.  When not visible, spacenav input is ignored.
* `/<server_type>/pov` [Quaternion] - Updates the server's self.pov
* `/spacenav/twist` [Twist] - Update the POV according to the angular values sent from the nav (adjust according to msg frequency). Count consecutive forward or backward movements and when threshold is met update the panoid.
* `/<server_type>/raw_metadata` [String] - Listens for raw metadata that needs
  to be converted and republished on `/<server_type>/metadata`.
* `/spacenav/joy` [Joy] - Used to listen for button presses.
* `/<server_type>/tilt_snappy` [Bool] - Sets tilt behavior. `true` means the tilt snaps back to zero (the default). `false` means the tilt is persistent and navigable.

### launcher (wrapper to clients)

##### Overview

Uses the sv launcher to start a browser with a sv instance. Add event listeners for POV, panoid msgs and links changed to the SV instance.  When changes come from POV and panoid topics via rosbridge emit a signal and let the sv instance update itself with the js api. The panoid of the current metadata doesn't match the actual current panoid, emit a signal to collect the metadata and publish it.

##### Parameters

* `~server_type` [string] - The type of server, `streetview` or `panoviewer`. Default: `streetview`
* `~url` [string] - The url of the webbapp. Default: `http://localhost:8008/lg_sv/webapps/client/index.html`
* `~fov` [string] - The field of view, how zoomed out we are from the images. Default: `28.125`
* `~pitch_offset` [float] - Pitch offset. Default: `0`
* `~show_links` [boolean] - Whether or not to show the chevrons that point to the direction of nearby panos. Only really useful on the center pano. Default: `False`
* `~show_attribution` [boolean] - Whether or not to show the attribution card on the top right of each screen. Should probably only be on the far left screen. Default: `False`
* `~yaw_offset` [float] - The number of screens over from the center screen. `left_one` would be `-1`, `right_two` would be `2`, etc. Default: `0`
* `~leader` [boolean] - Used on one screen (panoviewer only) to determine the master for video syncing. Default: `false`
* `~tilt` [boolean] - Whether or not (streetview only) tilt will be handled. Ignored by `lg_sv_nonfree`. Default: `false`
* `~depend_on_webserver` [bool] - Whether or not we need to wait for the
  webserver before starting the client. Default: `False`
* `~depend_on_rosbridge` [bool] - Whether or not we need to wait for rosbridge
  before starting the client. Default: `False`
* `~rosbridge_host` [string] - Address of the rosbridge host. Default: `127.0.0.1`
* `~rosbridge_port` [int] - Port number used by rosbridge. Default: `9090`
* `~rosbridge_secure` [string] - Whether or not rosbridge will be using
  TLS(SSL?). Default: `'false'`
* `~zoom` [bool] - Whether or not there will be zoom. Ignored by
  `lg_sv_nonfree`. Default: `'false'`
* `~initial_zoom` [int] - Starting zoom level. Only respected by `lg_sv`. Default: `3`

##### Published Topics

* `/<server_type>/pov` [Quaternion] - Update the current POV
* `/<server_type>/metadata` [String] - Json of the metadata that has the current panoid and adjacent panoids and their heading.

##### Subscribed Topics

* `/<server_type>/pov` [Quaternion] - Update sv instance's POV
* `/<server_type>/panoid` [String] - Update sv instances current pano with a new panoid to switch to.
* `/<server_type>/state` [ApplicationState] - Reacts to changes in state. Relies
  on ManagedAdhocBrowser's `handle_state_msg` function.

### clients

#### lg_sv

##### Overview

MapsV3api compliant 3d panoviewer. Uses the /streetview server_type.

##### Parameters

(see launcher above)

##### Published Topics

* `/streetview/pov` [Quaternion] - Updates the server's POV instance variable.
* `/streetview/metadata` [String] - Uses the metadata accessed via the maps API
  and turns it into JSON and publishes it so the server can access it.

##### Subscribed Topics

* `/streetview/panoid` [String] - Hides the titlecard until new metadata comes
  in. Changes the current pano on screen.
* `/streetview/metadata` [String] - Sets the titlecard.
* `/streetview/pov` [Quaternion] - Uses math to change the current pov.

#### panoviewer

##### Overview

Threejs implementation of a panoviewer. Works well for offline panoviewer
content. Might have Cross-origin problems when using images hosted online.

##### Parameters

(see launcher above)


##### Published Topics

* `/panoviewer/video_time` [float] - Publishes the current time in a video. Only
  used by the leader.

##### Subscribed Topics

* `/panoviewer/metadata` [String] - Sets the titlecard.
* `/panoviewer/video_time` [float] - Used to store the leader's current time in
  the video. Used by non-leaders.
* `/panoviewer/panoid` [String] - sets the current panoid (usually a local file)
  to the value attached.
* `/panoviewer/state` [ApplicationState] - Sets the current state. Videos will
  only play when the state == VISIBLE.
