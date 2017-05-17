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
