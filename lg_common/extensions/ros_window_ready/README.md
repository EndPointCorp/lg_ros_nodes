# Smooth transitions extensions

This extension handles an event when window is loaded
(`onDOMContentLoaded`) and sends `ros_instance_name` get parameter
to the `/director/window/ready` ROS topic.

### Get Parameters

- `ros_instance_name` window id. Key parameter for smooth transitions.
  Extension will not be loaded if it's missed.
- `rosbridge_host` host where ros bridge node are listening   
- `rosbridge_port` port where ros bridge node are listening   
- `rosbridge_secure` use or not ssl
- `use_app_event`
