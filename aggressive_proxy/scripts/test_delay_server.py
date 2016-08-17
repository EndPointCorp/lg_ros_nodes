#!/usr/bin/env python

# Starts up a slow HTTP server for use during automated testing.

# Authored by Adam Vollrath <adam@endpoint.com>

from time import time
import json

import rospy

from tornado.web import RequestHandler, Application
from tornado.ioloop import IOLoop

class DelayHandler(RequestHandler):
    def initialize(self, recent_requests):
        self.recent_requests = recent_requests

    def get(self, uri):
        # Record and timestamp this request in our recent requests.
        try:
            recent_requests[uri].append(time())
        except KeyError:
            recent_requests[uri] = [time(), ]

        delay = (float(uri) / 1000) #milliseconds

        # Wait until `delay` seconds since first request to this URI.
        self.sleep(recent_requests[uri][0], delay)

        self.set_status(200)

        # Return JSON list of request timestamps.
        self.write(json.dumps(recent_requests[uri]))
        self.finish()

    def sleep(self, start, delay):
        elapsed = time() - start
        remaining = delay - elapsed
        rospy.sleep(remaining)

if __name__ == '__main__':
    # Store state outside of the Web Application.
    recent_requests = {}

    # Configure an upstream HTTP server for testing.
    application = Application([
      (r"/delay/([0-9]+).html", DelayHandler, {'recent_requests': recent_requests}),
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
