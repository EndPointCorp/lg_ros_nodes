spacenav\_gmaps
-----------

ROS tools for navigating the Google Maps globe.

## Scripts/executables

### portal\_nav

Coordinates touchscreen (kiosk) navigation and SpaceNav joystick input into authoritative PoV for the globe.

##### Parameters

* `joystick_sensitivity` : `float` - Coefficient for the overall SpaceNav movement sensitivity.  Default is 1.0.

##### Topics

* `/portal_nav/kiosk_goto_pose` : `geometry_msgs/PoseStamped` - Outgoing position that the kiosk should move to immediately.

* `/portal_nav/display_goto_pose` : `geometry_msgs/PoseStamped` - Outgoing position that the display should move to immediately.

* `/portal_kiosk/current_pose` : `geometry_msgs/PoseStamped` - Incoming position change from the kiosk which can be from touchscreen control.

* `/joystick/twist` : `geometry_msgs/Twist` - Normalized SpaceNav axis values.
