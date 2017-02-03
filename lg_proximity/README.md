maxbotix
--------

ROS node for reading a Maxbotix proximity sensor.

### sender.py

Attaches to a proximity sensor via USB serial port and publishes proximity information.

##### Parameters

* `device_path` : Required.  Path to the serial device.  Remember that most devices will be available in `/dev/serial/by-id`.

* `baud_rate` : Baud rate for serial communications.  Defaults to 57600.

* `serial_device_attempts`: The number of attempts the node will make
  when trying to connect to the serial device. The number will be
  multiplied by 3 and it will wait 3 seconds, then 6 seconds then... all
  the way up to 3 * `serial_device_attempts`. Default: `10`

##### Topics

* `/proximity/distance` : `sensor_msgs/Range` - Distance (in meters) to nearest object in the sensor's field of view.

* `/proximity/presence` : `std_msgs/Bool` - Reports true if the sensor is reporting an object is in the field of view.  This is a "normalized" and somewhat delayed simplification for cases where we want to know if an object is present, but the theshold range is set by the device.
