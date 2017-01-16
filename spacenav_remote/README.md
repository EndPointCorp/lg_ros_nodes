## Remote control for spacenav

### Usage

1. Run spcenav_remote ros node on displaynode
2. Forward port `6564`
3. Run scrips/client.py on client computer


### ROS Nodes

1. spacenav_remote

Listens on port `6564` and publish on `/spacenav/twist` and `/spacenav/joy` ros topics

#### ROS Parameters

* `port` default: `6564` - port to listen on for spacenav msgs from `client.py`
* `topic` default: `/spacenav` - topic base bath for `<topic>/joy` and `<topic>/twist`
