# lg_nav_to_device

Ros node that will replay space navigator input events to a
`/dev/input/v_spacenav` device

## Software requirements

* spacenav node ROS node running somewhere in the network with
  spacenavigator attached to the machine it's running on

## Hardware requirements

* space navigator

## scripts

### device_writer.py

#### parameters

* `~scale` [int] - space navigator scale
* `~disable_activities` [str] - comma-separated list of activity types to disable nav on.  Default: `cesium,unity,sketchfab,streetview,panovideo`
* `~disable_states` [str] - comma-separated list of ApplicationState topics which would occlude Earth.  Workaround for free flight.  Default: `/streetview/state`

#### published topics

None

#### subscribed topics

* `/lg_twister/twist` - this script needs to subscribe to joystick
  data flowing on this topic

### navigation_debugger.py

Passive JSON-lines logger for intermittent SpaceNav-to-Earth stalls. It
correlates the physical, wrapper, transformed-touch, pointer, and mixed Twist
topics with the virtual `/dev/input/v_spacenav` output and `/earth/pose`.
It also mirrors every callback that currently writes the device writer's
last-callback-wins enable flag.

Run on the lead Earth machine, where the virtual input device exists:

```
rosrun lg_nav_to_device navigation_debugger.py
```

The default log is `/tmp/lg_navigation_debug.jsonl`. Important classifications
include:

* `physical_to_wrapper_gap`
* `wrapper_to_mixer_gap`
* `routing_gate_disabled`
* `virtual_device_disconnected`
* `virtual_device_no_output`
* `earth_pose_feedback_missing`
* `earth_not_responding_to_virtual_device`

Useful parameters:

* `~log_file` [str], default `/tmp/lg_navigation_debug.jsonl`
* `~device_path` [str], default `/dev/input/v_spacenav`
* `~input_threshold` [float], default `0.01`
* `~virtual_event_threshold` [int], default `2`
* `~activity_window` [float seconds], default `0.75`
* `~stall_timeout` [float seconds], default `1.5`
* `~summary_interval` [float seconds], default `10.0`
* `~writer_node` [str], default `/spacenav_emulator`; its routing parameters
  are used when the debugger has no private override
* `~disable_activities` and `~disable_states` should match `device_writer.py`
