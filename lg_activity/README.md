lg\_activity
---------------

ROS software for tracing activity and inactivity of Liquid Galaxy.
Tracking may be useful for statistics as well as for state-greedy
applications like screensaver (lg\_attract\_loop) or others.

## Hardware Requirements

None.

## Nodes

There's one node that's responsible for initialization, activity
tracking and activity messages emitting.

### tracker.py

Tracker is a configurable ROS node that initializes activity tracking
mechanisms. In general it parses the `activity_sources` parameter that
tells it which ROS topics are emitting user input devices messages. It
needs to know the message type that it should use to subscribe to the
topic as well as what should be extracted from user input device message
as a value (e.g. which attribute to compare).

![lg_activity
diagram](lg_activity.png "lg activity diagram")

Direct link to the diagram:
https://docs.google.com/drawings/d/1u2oJ-MyN6IwjE_-Oc1J7HWOvNe0uzMV816XNRqOh7sI/edit?usp=sharing


#### Parameters

* `~activity_timeout` [int] - amount of seconds needed for lg_activity
  to emit inactive message. This happens when all activity sources have
been inactive for more (or exactly) this amount of time. Default: `120`
* `~activity_topic` [string] - activity topic on which the True/False
  Bool messages are published. lg_activity will publish `True` when
there was activity on input devices (or `ActivitySources`) and `False`
if all activity sources were inactive for `activity_timeout` number of
seconds. Default: `/activity/active`
* `~memory_limit` [int] - maximum amount of bytes that single ActivitySource
  can keep in it's state for queued messages from activity source. Default:
  `102400`
* `~activity_sources` [string] - Default: `''`. string containing configuration
  for activity sources in following format:

`<topic_name>:<message_type>[-<slot.sub_slot.sub_sub_slot>]:<strategy>[-<value_min>,<value_max>]`

- `topic_name` - name of the ROS topic that activity source (e.g. user
  input) publishes to e.g. `/spacenav/twist`
- `message_type` - type of the message on `topic_name` topic in
  following format: <module_name>/<message_type> e.g.
`interactivespaces_msgs/GenericMessage` or `std_msgs/Bool`
- `slot.sub_slot` - single attribute of the message that should be
  considered during applying of `strategy`. Useful for complex messages
that you need to examine only one attribute from
- `strategy` - while computing active/inactive state on the basis of
  data flowing on `activity_topic`, one may choose 3 strategies:
 - `delta` - compares flowing messages - `active` is triggered when
   messages are differing from each other.
 - `value` - needs a `slot` to be defined (e.g. `angular` or `range`)
   and `value_min` as well as `value_max`. Activity is triggered when
value is **within** the `value_min`:`value_max` range
 - `activity`
- `value_min` - minimum value for the `value` strategy
- `value_max` - maximum value for the `value` strategy
NOTE: If slot/subslot value flowing on a topic is within the range then system is active

For example:

`/touchscreen/touch:interactivespaces_msgs/GenericMessage:delta`

Above example will compare messages flowing on `/touchscreen/touch`
topic that uses `interactivespaces_msgs.msg/GenericMessage` topic using
strategy `delta`.

Another example:

`/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5;/touchscreen/touch:interactivespaces_msgs/GenericMessage:activity`

Above example will subscribe to `/proximity_sensor/distance` and
`/touchscreen/touch`. Proximity sensor messages will be stripped down to
`range` value which will treat all values between 0 and 2.5 as active.
All messages flowing on /touchscreen/touch will trigger the activity
because of `activity` strategy.

More information about strategies can be found in details of this ROS node.

#### Published Topics

* `activity_topic` or `/activity/active` [`std_msgs/Bool`] - The current Earth view in degrees and meters ASL.

#### Subscribed Topics

* tracker.py creates ActivityTracker class that creates ActivitySource
  objects which subscribe to ROS topics specific to input that they are watching.
