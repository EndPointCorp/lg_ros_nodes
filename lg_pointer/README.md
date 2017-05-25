lg\_pointer
-----------

Pointing for Liquid Galaxy.  Pointing devices similar to WiiMote can be used to control a mouse pointer across the entire display span, or to navigate a 3D scene.

## Hardware

The ROS `wiimote_node` only seems to work with older USA-edition WiiMotes with MotionPlus integrated.  Maybe older devices too.  Definitely not with newer Wii U remotes.

## Nodes

### wiimote\_to\_twist.py

#### Overview

Turns WiiMote `State` data into orientation in the same axis space as the SpaceNavigator.  Intended to be muxed by an `lg_twister` `mux_twists` node.

Publishes `geometry_msgs/Twist` to `/lg_pointer/twist`.

#### Parameters

* `scale` [float] - Scale for movement.  Default: `0.05`

### wiimote\_to\_pointer.py

Turns WiiMote `State` into `lg_mirror` touchscreen events.

#### Parameters

* `device_id` [str] - `lg_mirror` event device name. Default: `wiimote`
* `viewports` [str] - Comma-separated list of viewports, from left to right. Required.
* `arc_width` [float] - Angle from the left side of the screen array cylinder to the right in radians. Default: `pi / 2` (90 degrees)


## Configuration

For full WiiMote control, you will want:

* `wiimote_to_twist.py`
* `wiimote_to_pointer.py` with `viewports` set to your main viewports from left to right.
* `lg_twister::mux_twists` including `/lg_pointer/twist` in `sources`.
* `lg_mirror::touch_receiver_node` on each main viewport with `device_id` matching your `wiimote_to_pointer.py` node.
* `wiimote::wiimote_node` with respawn, so the controller can be powered off and re-paired at will.

## User's Manual

### Pairing the WiiMote

When the system starts up, the WiiMote is not paired.  You will know that the WiiMote is not paired when the first blue LED at the bottom of the controller is not lit.

To pair the WiiMote, place it on a flat surface and press the 1+2 buttons at the same time.  You should see all four blue LEDs flashing for a few seconds.  Take care not to move the controller during this time.  If the first blue LED comes on solid, the WiiMote is paired.  Otherwise, try again.

### WiiMote Pointer

The WiiMote can control a mouse pointer across all main screens.  To begin controlling the pointer, point the remote at the middle of the center screen and press and hold the B button (the trigger).  As long as you are holding the B button, the WiiMote will move the mouse cursor.

To click, press the A button (with your thumb) while still holding down the B button.

### Navigating the Globe

The WiiMote can control the camera while viewing geographic content in Google Earth or Cesium.

Use the directional pad near the top of the WiiMote to pan the view.

Use the plus and minus buttons to change altitude.

Use the 1 and 2 buttons to change tilt.

### Turning Off the WiiMote

To save battery power, the WiiMote can be turned off.  Press and hold the power button at the top of the controller until all LEDs are turned off.  After turning off the WiiMote, it must be paired before use.

### Clearing IMU Drift

You may find while using the WiiMote pointer that the pointer is "drifting" in one direction regardless of how still it is being held.  This is due to error accumulating in the WiiMote's IMU sensor.  To reset the IMU, turn off the WiiMote with the power button and pair it again.

