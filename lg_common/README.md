lg\_common
----------

Common software for Liquid Galaxy ROS nodes.

#### ManagedApplication

Combined process and window manager.

#### ManagedWindow

Window manager.

#### SceneListener

Runs a callback upon scene messages.

### webapp module

See `examples/webapp.py` for an example of a Tornado Application with an integrated rosbridge web socket. If using a different web server, just be sure to hook its shutdown method to `rospy.on_shutdown()`.

#### RosbridgeWebSocket

A fork of the `rosbridge_server` web socket handler, for insertion in your own Tornado Applications.

This class depends upon the `rosbridge_library` package.

#### ros\_tornado\_spin()

This convenience method spins the Tornado `IOLoop` instance and shuts down when appropriate. It is intended as a replacement for `rospy.spin()`.

This method depends upon Tornado which is conveniently installed by the `rosbridge_server` package.

#### ros\_flask\_spin(app)

Similarly, spin a Flask app.
