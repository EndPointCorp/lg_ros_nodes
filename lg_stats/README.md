lg\_stats
---------

ROS software follows certain configurable ROS topics and submits
*stats* information into influxdb directly or via telegraf submission agent.

Spec / details EndPointCorp/lg_ros_nodes#126

## Strategies

- `default` - records a value of a message slot
- `default_session` - records a value of message slot (just like
  `default`) but adds a unique ID to emulate sessions style events
- `count` - returns amount of messages on a topic between resubmission
  periods - useful for touchscreen touches
- `count_nonzero` - same as `count` but ignores zeroes - useful for
  topics that constantly emit `0` during idle state e.g.
  `/spacenav/twist`
- `average` - calculates and records average value of a slot

## Software requirements

* InfluxDB Python client library: https://pypi.python.org/pypi/influxdb
* Nanotime Python library: https://pypi.python.org/pypi/nanotime


### Nodes

#### ros node lg_stats

Add description.

##### Parameters
List all parameters.

- `resolution` (default: `2`) - number of seconds for which a message is retained until the
  stats submission happens

- `submission_type` - can either be "InfluxDirect" when the lg_stats ROS node talks
  directly to the InfluxDB or "InfluxTelegraf" when lg_stats submits to the
  Telegraf service (Telegraf is a InfluxDB submission agent)

- `host` - server to which the stats messages are submitted to (according to
  the submission_type parameter, it's either the InfluxDB server or Telegraf server)

- `port` - just like with host ...

- `database` - name of the InfluxDB database, applicable only in the case of "InfluxDirect",
  the database is configured in Telegraf in case of "InfluxTelegraf"

- `activity_sources` - configuration of source topics - watched values
  the format is: source_topic:message_type:field_of_interest; reapeat ...
  e.g.: "/director/scene:interactivespaces_msgs/GenericMessage:message;/appctl/mode:appctl/Mode:mode;


##### Published Topics

`/lg_stats/debug` - debug information for lg_stats

##### Subscribed Topics

Subscribed topics are configued via `activity_sources` ROS parameter

###### Output message type description

-  `/lg_stats/debug`: `Event`
 - `string`: `system_name` - e.g. name of the LG system (such as lg-head-281)
 - `string`: `application` - e.g. application name of the source event (from the topics to which lg_stats
      subscribes to) (statistics Session message: application)
 - `string`: `src_topic` - name of the source topic which triggered this stats Event
 - `string`: `field_name` - name of the relevant message field (from the corresponding topic)
 - `string`: `type` - type of this output stats message (can either be 'event' or 'time-series')
 - `string`: `value` - copied source value of the corresponding relevant (watched) field_name
 - `string`: `influx` - stringified dictionary holding the data that is written into influxdb,
      has some constrains of the influxdb line_protocol (e.g. value type must be float)

Influxdb measurement corresponds to
    -watched ROS topic name
    -other watched metrics names (not yet implemented)
