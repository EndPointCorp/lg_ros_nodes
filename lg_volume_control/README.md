lg\_volume\_control
-------------------

Volume control over ROS

## Hardware requirements

* pulse audio enabled hardware

## Software requirements

* pulse audio...


### Nodes

#### volume_control

##### params

* `~default_sink`: The numbered sink that this node will work on.
  Default: `'1'`

#### Publishers

* `/volume/level`: The new volume level after a change has occured.
  Latching `std_msgs/UInt8`

#### Subscribers

* `/volume/increment`: The amount (positive or negative) to increment
  by. `std_msgs/Int8`
