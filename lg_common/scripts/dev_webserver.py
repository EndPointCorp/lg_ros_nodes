#!/usr/bin/env python

import os.path

import rospy
import rospkg
import tornado.web
import tornado.ioloop
from lg_common.helpers import get_params


class DevStaticHandler(tornado.web.StaticFileHandler):
    def set_default_headers(self):
        self.set_header('Access-Control-Allow-Origin', '*')

    def compute_etag(self):
        return None


if __name__ == '__main__':
    rospy.init_node('lg_dev_webserver')

    port = get_params('~port', 8008)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lg_common')
    share_path = os.path.abspath(os.path.join(pkg_path, os.pardir))
    rospy.loginfo('Serving files from {} on port {}'.format(share_path, port))

    handler = DevStaticHandler

    dev_webserver = tornado.web.Application([
        (r'/(.*)', handler, {'path': share_path}),
    ])
    dev_webserver.listen(port)

    ioloop = tornado.ioloop.IOLoop.instance()
    rospy.on_shutdown(ioloop.stop)
    ioloop.start()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
