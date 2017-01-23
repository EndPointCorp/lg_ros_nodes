#!/usr/bin/env python

import os.path

import rospy
import rospkg
import tornado.web
import tornado.ioloop
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'lg_dev_webserver'


class DevStaticHandler(tornado.web.StaticFileHandler):
    def set_default_headers(self):
        self.set_header('Access-Control-Allow-Origin', '*')

    def compute_etag(self):
        return None


def main():
    rospy.init_node(NODE_NAME)

    port = rospy.get_param('~port', 8008)

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


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
