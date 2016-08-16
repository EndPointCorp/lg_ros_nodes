#!/usr/bin/env python

# Authored by Adam Vollrath <adam@endpoint.com>

# Inspiration from and credit to:
# http://dumpz.org/12291/
# https://groups.google.com/forum/#!msg/python-tornado/TB_6oKBmdlA/Js9JoOcI6nsJ

# Some public design discussion:
# https://groups.google.com/forum/#!topic/python-tornado/lhyGhLZQIxY

import rospy
import tornado.web
import tornado.ioloop
import tornado.httpclient
from aggressive_proxy import ProxyHandler

if __name__ == "__main__":
    # use `pycurl`, an external dependency
    tornado.httpclient.AsyncHTTPClient.configure("tornado.curl_httpclient.CurlAsyncHTTPClient")

    # Single client instance
    client = tornado.httpclient.AsyncHTTPClient(max_clients=1000)

    rospy.init_node('aggressive_proxy')

    proxy_port = int(rospy.get_param('~proxy_port', 9900))
    application = tornado.web.Application([
        (r"/(.*)", ProxyHandler, {'client': client}),
    ], debug=True)
    application.listen(port=proxy_port)

    ioloop = tornado.ioloop.IOLoop.current()
    rospy.on_shutdown(ioloop.stop)
    ioloop.start()
