lg\_twister
-----------

Data muxer for joystick-like input devices.

This abstraction lets us handle a common input stream for Space Navigators, WiiMote-like pointers, and beyond.

## Nodes

### mux\_twists

Performs twist muxing on any number of sources.  Publishes the clamped sum of all sources on a regular interval.

Publishes `geometry_msgs/Twist` on `/lg_twister/twist`.

#### Parameters

* `sources` [str] - Comma-separated list of `geometry_msgs/Twist` topics to mux.  Required.
* `tick_rate` [float] - Publish timer frequency in Hz.  Default: `65`
* `axis_limit` [float] - Upper and lower limit for each axis.  Default: `sqrt(2) / 2`
* `age_limit` [float] - Maximum sample age in seconds.  Samples older than this will be ignored.  Default: `1.0`
