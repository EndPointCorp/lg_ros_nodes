lg\_mirror
---------

Software for mirroring portions of a screen on other portions of a screen.

## Nodes

### capture\_viewport

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

### playback

#### Overview

Manages playback windows for the given viewport.

There is no limit to how many playback instances can be spawned by a scene change.

However, all playback instances for the same source viewport must have equal dimensions.

* `viewport` [str] - Viewport to be managed by this node. Required.
* `display` [str] - Xorg DISPLAY to use. Default: Value of DISPLAY environment.

#### Subscribed Topics

* `/director/scene` [GenericMessage] - Director scene messages.
