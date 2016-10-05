lg\_mirror
----------

Software for mirroring portions of a screen on other portions of a screen.

## Nodes

### capture\_viewport\_node

#### Overview

Manages capture processes for the given viewport.

In practice, this will start a GStreamer `ximagesrc`, encode the video stream to VP8, payload as RTP, and emit datagrams. The target host is automatically determined by the viewport key.

Each viewport may only have one capture process in a scene.

All playback instances sourcing the same viewport must have identical dimensions.

#### Parameters

* `viewport` [str] - Viewport to be managed by this node. Required.
* `display` [str] - Xorg DISPLAY to use. Default: Value of DISPLAY environment.
* `show_pointer` [bool] - Show the mouse pointer in the capture. Default: `false`
* `framerate` [int] - Capture framerate in FPS. Default: `30`
* `max_quantizer` [int] - Minimum image quality for VP8 encoding. Lower is higher quality `[0-63]`. Default: `24`
* `janus_port` [int] - RTP port for this viewport. See Janus Gateway config for port:viewport mapping. Required.
* `/lg_mirror/janus_stream_host` [str] - Target host for RTP datagrams. Can be any uni, multi, or broadcast address. Required.

#### Subscribed Topics

* `/director/scene` [GenericMessage] - Director scene messages.

### capture\_webcam\_node

Constantly captures video stream from a V4L2 device.

#### Parameters

* `device` [str] - Path to V4L2 device. Default: `/dev/video0`
* `width` [int] - Width of output video stream in pixels. Default: auto selected by V4L2.
* `height` [int] - Height of output video stream in pixels. Default: auto selected by V4L2.
* `framerate` [int] - Frame rate of output video stream in frames per second. Default: auto selected by V4L2.
* `max_quantizer` [int] - Maximum quantization level. Lower is higher quality. Default: `60`
* `target_bitrate` [int] - Target video bitrate in bits/sec. Default: `768000`

### playback\_node

**This node is presently deprecated, use a browser activity to launch playback browsers.**

#### Overview

Manages playback windows for the given viewport.

There is no limit to how many playback instances can be spawned by a scene change.

However, all playback instances for the same source viewport must have equal dimensions.

#### Parameters

* `viewport` [str] - Viewport to be managed by this node. Required.
* `display` [str] - Xorg DISPLAY to use. This is for testing. Default: Value of DISPLAY environment.
* `/lg_mirror/janus_rest_uri` [str] - URI to Janus Gateway REST API. Required.

#### Subscribed Topics

* `/director/scene` [GenericMessage] - Director scene messages.

### touch\_router\_node

#### Overview

A required router for touch event redirection. This mechanism allows mirror windows to request that touch events be routed to the source viewport instead of the default route during a scene.

During a scene with no touch routing mirror activities (e.g. blank scene), the router will direct touch events to its default viewport (typically, the touchscreen). If a mirror activity is detected in a scene with the `route_touch` activity config key set to `true`, touches will instead be routed to the source viewport of that activity. Multiple activities in a scene may request touch routing, and they will all receive touches.

The purpose of this is to mirror an application from main screens to the touchscreen and route touches back to the application, as a quick way to integrate an existing un-synchronized application.

#### Parameters

* `default_viewport` [str] - An optional default viewport for touch routing. Touches will be routed to this viewport's receiver unless otherwise requested by a scene.

#### Subscribed Topics

* `/director/scene` [GenericMessage] - Director scene messages.

#### Published Topics

* `/lg_mirror/active_touch_routes` [lg\_common/StringArray] - A list of viewports which should be receiving touch events.

### touch\_sender

#### Overview

Reads touch events from a touchscreen and publishes them.

#### Parameters

* `device_path` [str] - Path to the event device for the touchscreen. This will typically be a device or symlink under `/dev/input`. Required.

#### Published Topics

* `/lg_mirror/touch_events` [EvdevEvents] - Touch events.

#### Services

* `/lg_mirror/device_info` [EvdevDeviceInfo] - Profile of the device's evdev properties, used for accurate cloning.

### touch\_receiver

#### Overview

Creates a uinput clone of the sending device, maps it to a viewport, and conditionally subscribes to events.

#### Parameters

* `viewport` [str] - The viewport to be managed by this receiver.

#### Subscribed Topics

* `/lg_mirror/touch_events`
* `/lg_mirror/active_touch_routes`

## Configuration

For full capability, you will need to configure:

* A separate Janus Gateway instance with VP8/RTP streaming agents and REST API.
* Global parameter `/lg_mirror/janus_rest_uri` (see `playback_node`)
* Global parameter `/lg_mirror/janus_stream_host` (see `capture_viewport_node`)
* A single `touch_sender` for the touchscreen device, on its host.
* A single `touch_router_node`, anywhere on the graph, defaulting to the touchscreen viewport.
* A `touch_receiver` for each viewport.
* A `capture_viewport_node` for each viewport.
* An `lg_common::dev_webserver.py` for each host.
* An `lg_common::adhoc_browser.py` for each viewport.

## Playback Webapp

The playback webapp lives in `/lg_mirror/webapps/playback/index.html` and requires a couple of params:

* `janusUrl` : url to Janus Gateway REST API
* `streamDescription` : description of the stream (viewport name)

The webapp will connect to Janus and look for a stream whose description matches `"Mirror: %viewport%"`

## Development/Testing

There are rostests, nosetests, and gtests for this package.  Run them with `catkin_make run_tests_lg_mirror && catkin_test_results`.

A [test Janus Gateway instance can be run in docker](https://github.com/EndPointCorp/docker-janus).  Janus Gateway is required for mirroring.

A combination of launch file and USCS message script can be used for testing mirroring.

```
roslaunch launch/mirror_dev.launch
python test/test_mirror_scene.py
python test/blank_scene.py
```

## Authorship in an Alternate History

Prior to this git tree, authors were:

* Matt Vollrath
* Jacob Minshall - EvdevDeviceInfo service
