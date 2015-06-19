#!/usr/bin/env python

import rospy
from lg_common.webapp import ros_flask_spin
from flask import Flask
from std_msgs.msg import String

"""
This is an example of a custom Flask web application.
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


def main():
    stringer = StringHolder()

    rospy.init_node('test_webapp')

    port = rospy.get_param('~port', 9000)

    # Messages handled by MainHandler's StringHolder instance.
    rospy.Subscriber(
        '/test_webapp/str',
        String,
        stringer.handle_string_message
    )

    app = Flask(__name__)
    @app.route(r'/([^/]+)')
    def get(slug):
        s = stringer.string
        rospy.loginfo('GET /{} ({})'.format(slug, s))

        if slug == s:
            res = 'You asked for {} and got it!\n'.format(slug, s)
        else:
            res = 'You asked for {} (but I was expecting {})\n'.format(slug, s)

        return res

    # This method replaces rospy.spin() and blocks until rospy is shutdown.
    ros_flask_spin(app, port=port)


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
