RFID\_Scanner
-------

Serial port listener for our rfid scanners. Has applications beyond just rfids though.

### reader.py

This package listens in on a port, and echos it out on a parameterized topic
stripped of non al-num characters. A notification will be sent out when
an rfid is scanned and when the mode changes (between setting /
recording the state).

To set instead of view rfids, publish `true` on `/rfid/mode`

##### Parameters

* `device_path`: Path to the device. Default: `/dev/rfid_scanner`

* `baudrate`: Baudrate for the device. Default: `9600`

* `pub_topic`: Topic the cleaned up output is output on. Default:
  `/rfid/uscs/scan` (Publisher topic)

* `notification_topic`: The topic which notifications are sent (topic
  should accept strings and expect json w/ a title and message at least.
  Default `/portal_launcher/notification` (Publisher topic)

* `set_topic`: Topic that is published on when paring an rfid with the
  current state. The current rfid will be published on that topic.
  Default: `/rfid/set` (Publisher topic)


### sqlite_uscs_storage.py

This listens in on two topics and will read the state that matches an
rfid, or update the database to pair the current state with the rfid
found in the message. It writes everything to an sqlite database.

#### Parameters

* `database_path`: The path to the database. Default
  `/home/lg/rfid/rfid_storage.db`

* `scan_topic`: The topic that scanned rfids come in on. The state needs
  to be set with the data associated with the rfid passed on this topic.
  Default: `/rfid/uscs/scan` (Subscriber topic)

* `state_set_topic`: The topic that the state_setter will listen in on
  for a uscs message in json form. Default: `/state_setter/set_state`
  (Publisher topic)

* `error_topic`: Topic where error messages will be published on.
  Default: `/info/error` (Publisher topic)

* `update_topic`: The topic that is published on when an rfid needs to
  be associated with the state passed on this topic. Default
  `/rfid/uscs/update` (Subscriber topic)
