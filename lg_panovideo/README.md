lg\_panovideo
-------------

ROS software for displaying panoramic videos in cylindrical projection.

## Hardware requirements

* accelerated graphics card
* decent CPU

## Software requirements

* Google Chrome

## Nodes

### launcher

##### Overview

Launches a panovideo browser when a panovideo scene is loaded.

##### Parameters

* `~viewports` [str] - Comma-separated list of viewports to cover.  Required.
* `~url` [str] - Override url to webapp.  Default: `http://localhost:8008/lg_panovideo/webapps/panovideosync/index.html`
* `~fov` [float] - Horizontal field of view per panel in degrees.  Default: `30`
* `~yaw_offsets` [str] - Comma-separated list of yaw offset values in degrees.  Default: `0`
* `~leader` [bool] - If true, this instance will play audio and broadcast sync timing.  Default: `false`
* `~clock_addr` [str] - Address to the `ws_distributor` web socket server.  Default: `ws://localhost:9091`
* `~kiosk` [bool] - Launch browsers in kiosk mode.  Default: `true`

### ws\_distributor

##### Overview

A high speed web socket relay for clock sync.

##### Parameters

* `~port` [int] - Port to listen on.  Default: `9091`
