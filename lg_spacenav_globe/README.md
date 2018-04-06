lg\_spacenav\_globe
-----------

ROS tools for navigating the Google Maps globe.

## Scripts/executables

### lg\_spacenav\_globe

Coordinates touchscreen (kiosk) navigation and SpaceNav joystick input into authoritative PoV for the globe.

##### Parameters

* `joystick_sensitivity` : `float` - Coefficient for the overall SpaceNav movement sensitivity.  Default is `1.0`.

* `spacenav_scale` : `float` - Compatibility for ROS SpaceNav driver full\_scale param.  Use 350.0 for legacy full\_scale of 1.0, or 0.7 for new full\_scale of 512.0.  Default: `350.0`

##### Topics

* `/portal_nav/kiosk_goto_pose` : `geometry_msgs/PoseStamped` - Outgoing position that the kiosk should move to immediately.

* `/portal_nav/display_goto_pose` : `geometry_msgs/PoseStamped` - Outgoing position that the display should move to immediately.

* `/portal_kiosk/current_pose` : `lg_spacenav_globe/PortalPose` - Incoming position change from the kiosk which can be from touchscreen control.

* `/joystick/twist` : `geometry_msgs/Twist` - Normalized SpaceNav axis values.
