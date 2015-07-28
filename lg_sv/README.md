lg\_sv
---------

ROS software for running a streetview client/server.

## Nodes

### server

##### Overview

Creates rostopics to communicate/interact with the sv clients. Handles the spacenav input and publishes the changes to POV and triggers panoid changes.

##### Parameters

* `DEFAULT_TILT_MAX` [float] - Maximum value in degrees which the SV client can tilt. Default: `80`
* `DEFAULT_TILT_MIN` [float] - Minimum value in degress which the SV client can tilt. Default: `-80`
* `DEFAULT_NAV_SENSITIVITY` [float] - Value to adjust space nav angular changes by. Default: `1.0`
* `DEFAULT_NAV_INTERVAL` [float] - Messages from the space nav are adjusted in relation to their frequency when updating the POV. An increase in frequency will reduce the impact of each message on the POV.  Default: `0.1`

##### Published Topics

* `/streetview/panoid` [string] - The current sv panoid
* `/streetview/pov` [Quaternion] - The current pov of the sv instance

##### Subscribed Topics

* `/streetview/location` [pose2d] - Lookup a panoid for the location and publish it if different than the current panoid.
* `/streetview/metadata` [string] - JSON representation of the sv metadata collected by the clients.  Save the list of links and their headings.  On move.forward compare current heading with list of links and find the link with the closest heading and update panoid.
* `/streetview/panoid` [string] - Set the servers self.panoid
* `/streetview/state` [ApplicationState] - The state of the sv application
* `/streetview/pov` [Quaternion] - Update the server's self.pov
* `/spacenav/twist` [Twist] - Update the POV according to the angular values sent from the nav (adjust according to msg frequency). Count consecutive forward or backward movements and when threshold is met update the panoid.

### client

##### Overview

Uses the sv launcher to start a browser with a sv instance. Add event listeners for POV, panoid msgs and links changed to the SV instance.  When changes come from POV and panoid topics via rosbridge emit a signal and let the sv instance update itself with the js api. The panoid of the current metadata doesn't match the actual current panoid, emit a signal to collect the metadata and publish it.

##### Published Topics

* `/streetview/pov` [Quaternion] - Update the current POV
* `/streetview/metadata` [String] - Json of the metadata that has the current panoid and adjacent panoids and their heading.

##### Subscribed Topics

* `/streetview/pov` [Quaternion] - Update sv instance's POV
* `/streetview/panoid` [String] - Update sv instances current pano with a new panoid to swtihc to.
