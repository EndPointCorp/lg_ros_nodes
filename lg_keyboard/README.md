lg\_keyboard
------------

ROS software handling displying virtual keyboards on the Liquid Galaxy system.

The current implementation utilizes system application `onboard` but it's
likely not the least virtual keyboard implementation within this ROS package.

Spec / details: EndPointCorp/lg_ros_nodes#127

### Nodes

#### ros node lg\_keyboard


### Parameters

* `~viewport` [string] - viewport the onboard keyboard appears on. Default: none
* `~config_path` [string] - path to an onboard dconf file to load instead of the packaged default. Default: none
* `~default_viewport` [string] - viewport the router falls back to when a scene does not name one. Default: none


### Published Topics

