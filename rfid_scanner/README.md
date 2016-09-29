RFID\_Scanner
-------

Serial port listener for our rfid scanners. Has applications beyond just rfids though.

### listener.py

This package listens in on a port, and echos it out on a parameterized topic stripped of non al-num characters.

##### Parameters

* `device_path`: Path to the device. Default: `/dev/rfid_scanner`

* `baudrate`: Baudrate for the device. Default: `9600`

* `pub_topic`: Topic the cleaned up output is output on. Default: `/rfid/scan`
