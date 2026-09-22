spacenav\_wrapper
-----------------

This will act as a wrapper to the spacenav. It will be able to translate
bad data into good data!


## scripts

### spacenav_wrap

A wrapper around the spacenav node that will clean up messages that fall
into the gutter value's domain, and issue relaunches when it senses one
is needed.

#### parameters

* `~root_topic` [string] - the topic which the spacenav is echoed out
  on. Default: `/spacenav_wrapper`
* `~gutter_value` [float] - the smallest value not thrown into the
  gutter. Default: `0.0` Default is 0 so the node does not affect anyone
  unless specifically set.
* `~buffer_size` [int] - the number of spacenav messages to store to
  check for the need of a relaunch. If all values in the array of
  `buffer_size` are the same, a relaunch is initiated. Default: `200`
* `~relaunch` [bool] - whether or not a relaunch command will be
  initiated. Default: `false`

#### topics

* `<root_topic>/twist` [geometry\_msgs/Twist] - All twist messages from
  the spacenav.


### spacenav_antispin

Rezeroes the spacenav when it reports the same non-zero value for too
long, which is how a stuck or drifting puck presents.

#### parameters

* `~spacenav_topic` [string] - topic the twist messages are read from. Default: `/spacenav/twist`
* `~seconds_before_rezero` [int] - seconds of unchanging input before a rezero is issued. Default: `30`
* `~rate` [int] - polling rate in Hz. Default: `4`

### spacenav_rezero

A node to rezero the spacenav.

#### topics

* `/rezero` [std\_msgs/String] - Any message sent on this topic will
  initiate a spacenav rezero.
