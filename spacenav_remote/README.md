## Remote control for spacenav

### Pre requirements

On a **client** side install `spnav` python package

    `pip install spnav`


### Usage

1. Run spcenav_remote ros node on **displaynode**

    rosrun spacenav_remote server.py

2. Open the port 6564 on the remote system by adding the below to /etc/iptables.conf:

    -A INPUT -p tcp -s 10.42.0.0/16 --dport 6564 -j ACCEPT

3. Forward port `6564`
4. Run scrips/client.py on **client** computer


### ROS Nodes

1. spacenav_remote

Listens on port `6564` and publish on `/spacenav/twist` and `/spacenav/joy` ros topics

#### ROS Parameters

* `port` default: `6564` - port to listen on for spacenav msgs from `client.py`
* `topic` default: `/spacenav` - topic base bath for `<topic>/joy` and `<topic>/twist`
