lg\_offliner
------------

ROS software checking internet connection based on configurable network metrics.
The metrics is a ROS node configurable set of network unix commands such as curl or ping.

All commands must fail (return non zero status) in two consecutive rounds in order to
pronounce offline status. Online status is pronounced when at least one command
succeeds (zero exit status).

Spec / details: EndPointCorp/lg_ros_nodes#113

### Nodes

#### ros node lg_offliner

The only ROS node of the package. Takes care for reading the configuration and
instantiating the checker class with options read from the ROS node configuration.

The class handles a background thread which periodically runs the set of
defined unix commands.


### Parameters

- `check_every_seconds_delay` (default: `30`) - the delay for which the background
   thread sleeps, the delay between subsequent rounds of unix network commands execution

- `checks` - the list of unix network commands used to determine the network status
 
- `send_on_online` - topics, message types and predefined messages to be sent when
   becoming online (syntax like 'activity config')

- `send_on_offline` - topics, message types and predefined messages to be sent when
   becoming offline (syntax like 'activity config')


### Published Topics

- `/lg_offliner/debug` - debug information for lg_offliner, outputs results of commands execution, etc

- `/lg_offliner/status` - sends True when becoming offline or False when becoming online,
   it's offline status Flag sent on the status transition

- on online topics as configured via `send_on_online`

- on offline topics as configured via `send_on_offline`

### ROS services

`lg/offliner/status` - return True/False, True when being offline
