import rospy

from functools import partial
from tornado.ioloop import IOLoop
from tornado.websocket import WebSocketHandler

# use a reserved code for protocol instantiation failure
PROTO_OPEN_FAILED = 4000


def ros_tornado_spin():
    """ Runs the Tornado IOLoop. Shuts down when rospy shuts down. """
    ioloop = IOLoop.instance()
    rospy.on_shutdown(ioloop.stop)
    ioloop.start()


def ros_flask_spin(app):
    """ Runs Flask. Shuts down when rospy shuts down. """
    from flask import request
    shutdown = request.environ.get('werkzeug.server.shutdown')
    rospy.on_shutdown(shutdown)
    app.run()


class RosbridgeWebSocket(WebSocketHandler):
    """ This was ripped off from rosbridge and should be worked back in. """
    client_id_seed = 0
    clients_connected = 0

    def open(self):
        cls = self.__class__

        try:
            from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
            self.protocol = RosbridgeProtocol(cls.client_id_seed)
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            assert hasattr(cls, 'client_id_seed')
            assert hasattr(cls, 'clients_connected')
        except Exception as exc:
            rospy.logerr('Unable to accept incoming connection.  Reason: %s', str(exc))
            self.close(code=PROTO_OPEN_FAILED, reason=str(exc))
            return

        cls.client_id_seed += 1
        cls.clients_connected += 1
        rospy.loginfo('Client connected.  %d clients total.', cls.clients_connected)

    def on_message(self, message):
        self.protocol.incoming(message)

    def on_close(self):
        if self.close_code == PROTO_OPEN_FAILED:
            return

        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        rospy.loginfo('Client disconnected.  %d clients total.', cls.clients_connected)

    def send_message(self, message):
        IOLoop.instance().add_callback(partial(self.write_message, message))

    def check_origin(self, origin):
        return True

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
