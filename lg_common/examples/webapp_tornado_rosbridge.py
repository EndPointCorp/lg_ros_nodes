#!/usr/bin/env python

import rospy
from lg_common.webapp import RosbridgeWebSocket, ros_tornado_spin
from tornado.web import Application, RequestHandler
from std_msgs.msg import String

"""
This is an example of a custom Tornado web application with a dedicated
rosbridge server listening on the same port.

You can make a web socket connection on `/websocket`.

Any other request will compare the path to a string held in an instance of
`StringHolder`. The expected string can be changed by publishing a message to
`/test_webapp/str`.
"""


class StringHolder:
    """
    This is any old state-keeping class. Nothing special about it.
    """
    def __init__(self):
        self.set_string('init')

    def set_string(self, s):
        self.string = s
        rospy.loginfo('Now listening for {}'.format(self.string))

    def handle_string_message(self, msg):
        self.set_string(msg.data)


class MainHandler(RequestHandler):
    """
    This is a handler that uses an instance of StringHolder to fulfill
    requests. Since a new instance of MainHandler will be instantiated upon
    each request, the instance of StringHolder is a class attribute.
    """
    stringer = StringHolder()

    def get(self, slug):
        # Grab the StringHolder instance.
        stringer = self.__class__.stringer

        s = stringer.string
        rospy.loginfo('GET /{} ({})'.format(slug, s))

        if slug == s:
            res = 'You asked for {} and got it!\n'.format(slug, s)
        else:
            res = 'You asked for {} (but I was expecting {})\n'.format(slug, s)

        self.finish(res)


def main():
    rospy.init_node('test_webapp')

    port = rospy.get_param('~port', 9000)

    # Messages handled by MainHandler's StringHolder instance.
    rospy.Subscriber(
        '/test_webapp/str',
        String,
        MainHandler.stringer.handle_string_message
    )

    # Tornado Application combining a rosbridge web socket and our MainHandler.
    app = Application([
        (r'/websocket', RosbridgeWebSocket),
        (r'/([^/]+)', MainHandler),
    ])

    app.listen(port)

    # This method replaces rospy.spin() and blocks until rospy is shutdown.
    ros_tornado_spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
