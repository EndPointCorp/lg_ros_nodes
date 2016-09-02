lg\_mirror
---------

Software for mirroring portions of a screen on other portions of a screen.

## Nodes

### capture\_viewport\_node

#### Overview

Manages capture processes for the given viewport.

In practice, this will start a GStreamer `ximagesrc`, encode the video stream to MJPEG, payload as RTP, and emit datagrams. The target host is automatically determined by the viewport key.

Each viewport may only have one capture process in a scene.

All playback instances sourcing the same viewport must have identical dimensions.

#### Parameters

* `viewport` [str] - Viewport to be managed by this node. Required.
* `display` [str] - Xorg DISPLAY to use. Default: Value of DISPLAY environment.
* `quality` [int] - JPEG encoding quality [0, 100]. Higher is more quality. Default: `85`
* `show_pointer` [bool] - Show the mouse pointer in the capture. Default: `false`
* `host` [str] - Target host for RTP datagrams. Can be any uni, multi, or broadcast address. Default: automatically determined by viewport.

#### Subscribed Topics

* `/director/scene` [GenericMessage] - Director scene messages.

### playback\_node

#### Overview

Manages playback windows for the given viewport.

There is no limit to how many playback instances can be spawned by a scene change.

However, all playback instances for the same source viewport must have equal dimensions.

* `viewport` [str] - Viewport to be managed by this node. Required.
* `display` [str] - Xorg DISPLAY to use. This is for testing. Default: Value of DISPLAY environment.

#### Subscribed Topics

* `/director/scene` [GenericMessage] - Director scene messages.

### touch\_router\_node

#### Overview

A required router for touch event redirection. This mechanism allows mirror windows to request that touch events be routed to the presentation viewport instead of the default route during a scene.

During a scene with no touch routing mirror activities (e.g. blank scene), the router will direct touch events to its default viewport (typically, the touchscreen). If a mirror activity is detected in a scene with the `route_touch` activity config key set to `true`, touches will instead be routed to the presentation viewport of that activity. Multiple activities in a scene may request touch routing, and they will all receive touches.

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

* `/lg_mirror/event` [EvdevEvents] - Touch events.

#### Services

* `/lg_mirror/device_info` [EvdevDeviceInfo] - Profile of the device's evdev properties, used for accurate cloning.

### touch\_receiver

#### Overview

Creates a uinput clone of the sending device, maps it to a viewport, and conditionally subscribes to events.

#### Parameters

* `viewport` [str] - The viewport to be managed by this receiver.

#### Subscribed Topics

* `/lg_mirror/event`
* `/lg_mirror/active_touch_routes`

## Configuration

Typically, you will need to configure:

* A single `touch_sender` for the touchscreen device, on its host.
* A single `touch_router_node`, anywhere on the graph, defaulting to the touchscreen viewport.
* A `touch_receiver` for each viewport.
* A `capture_viewport_node` for each viewport.
* A `playback_node` for each viewport.
