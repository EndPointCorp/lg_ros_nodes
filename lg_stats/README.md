lg\_stats
---------

ROS software for listening on certain configurable ROS topics and submitting
*stats* information into influxdb via telegraf submission agent
Details lg_ros_nodes: #126

Library dependency:
https://pypi.python.org/pypi/influxdb

### Nodes

#### ros node lg_stats

Add description.

##### Parameters
List all parameters.

##### Published Topics
/lg_stats/debug

##### Subscribed Topics
List of possible subscribed topics - it's configurable via roslaunch file.

###### Ootput message type description
 /lg_stats/debug Event
 
string system_name - e.g. name of the LG system (such as lg-head-281)
string application - e.g. application name of the source event (from the topics to which lg_stats
    subscribes to) (statistics Session message: application)
string src_topic - name of the source topic which triggered this stats Event
string field_name - name of the relevant message field (from the corresponding topic)
string type - type of this output stats message (can either be 'event' or 'time-series')
string value - copied source value of the corresponding relevant (watched) field_name
string influx - stringified dictionary holding the data that is written into influxdb,
    has some constrains of the influxdb line_protocol (e.g. value type must be float)
    
Infludb measurement corresponds to
    -watched ROS topic name
    -other watched metrics names (not yet implemented)