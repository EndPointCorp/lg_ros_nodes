lg_replay
---------

ROS node used for replaying device events on a ROS topic

## Hardware requirements

* any input device like touchscreen or spacenavigator

## Software requirements

None

## Nodes

### replay.py

#### Parameters

* `topic_name` [string] (default: `None` - mandatory) - name of the
  topic that events flowing on the device will be replayed to
* `device_path` [string] (default: `None` - `device_name` or `device_path`
  mandatory) - path to the device to listen on.
* `event_ecode` [string] (default: `EV_KEY` optional) - type of the
  UInput event to filter by and replay
* `user` [string] (default: `lg` optional) - username to change perms
  for UInput device to - NOTE: this user must have sudo perms in order
  to be able to change perms
* `group` [string] (default: `lg` optional) - group of the user to change perms
  for UInput device to - NOTE: this user must have sudo perms in order
  to be able to change perms
* `device_name` [string] (default: `None` - `device_name` or `device_path`
  mandatory) - name of the device that replay.py will attach to. You can find out the name of the
 device by executing python-evdev code like below:

```python
from evdev import InputDevice
dev = InputDevice('/dev/input/event7')
dev.name
```

#### Published topics

* topic configured by `topic_name` - good practice is to set it to
  `/lg_replay/<shot_device_name>`
