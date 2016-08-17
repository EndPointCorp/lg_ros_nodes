#!/usr/bin/env python
# Starts up a slow HTTP server for use during automated testing.

# Authored by Adam Vollrath <adam@endpoint.com>

import rospy

from tornado.web import RequestHandler, Application
from tornado.ioloop import IOLoop

class DelayHandler(RequestHandler):
    def get(self, args=None):
        self.set_status(200)
        self.write(args[0])
        self.finish()

if __name__ == '__main__':
    # Configure an upstream HTTP server for testing.
    application = Application([
      (r"/delay/([0-9]+).html", DelayHandler),
    ], debug=True)

    # Should match parameters for the aggressive proxy node during testing.
    rospy.init_node("test_delay_http_server")
    delay_port = int(rospy.get_param('~port'))
    application.listen(port=delay_port)

    # Start up the upstream HTTP server.
    ioloop = IOLoop.current()
    rospy.on_shutdown(ioloop.stop)
    ioloop.start()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
