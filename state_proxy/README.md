state\_proxy
-------

Tracks the state and sets the state.

### state\_tracker.py

This node tracks the current state and updates the USCS message
accordingly to reflect any url changes since the state was set.

##### Parameters

* `current_state_topic` - Topic to publish the current state on.

* `update_rfid_topic` - The topic that spreadsheet update requests will
  come from. Default `/rfid/uscs/update`

* `tactile_flag` - Special flag to append to tactile related urls.
  Default: `''`

### state\_setter.py

This node handles setting the state when requested via a service.

#### Service

* `/state_setter/desired_state` [`DesiredState`] - sets the state to the
  DesiredState via publishing to `/director/scene`
