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
* `~yaw_offset` [float] - Yaw offset for this viewport, in degrees. Default: `0`
* `~depend_on_webserver` [bool] - Wait for the webserver to answer before launching. Default: `false`
* `~depend_on_rosbridge` [bool] - Wait for rosbridge to answer before launching. Default: `false`
* `~rosbridge_host` [string] - Rosbridge host embedded in the client url. Default: `localhost`
* `~rosbridge_port` [int] - Rosbridge port embedded in the client url. Default: `9090`
* `~rosbridge_secure` [string] - Use TLS for rosbridge in the client url. Default: `'false'`

### ws\_distributor

##### Overview

A high speed web socket relay for clock sync.

##### Parameters

* `~port` [int] - Port to listen on.  Default: `9091`
