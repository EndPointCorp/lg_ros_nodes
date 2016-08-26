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
- `use_app_event` instead of `onDOMContentLoaded` wait for
  app generated event.

### Application generated message

Some applications loads assets and do some initialization,
asynchronously and `onDOMContentLoaded` will generate event
too early.

To avoid that, add `use_app_event=1` get parameter.
To send the message broadcast dom event for window:

```javascript
window.postMessage({ type: "DIRECTOR_WINDOW_READY" }, "*");
```
