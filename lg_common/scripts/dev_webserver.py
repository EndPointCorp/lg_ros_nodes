#!/usr/bin/env python3

import logging
import os.path

import rospy
from tornado.log import access_log
import tornado.web
import tornado.ioloop
from lg_common.helpers import get_package_path
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'lg_dev_webserver'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)

# Reduce tornado access log verbosity
access_log.setLevel(logging.WARNING)


class DevStaticHandler(tornado.web.StaticFileHandler):
    def set_default_headers(self):
        self.set_header('Access-Control-Allow-Origin', '*')

    def compute_etag(self):
        return None


def main():
    rospy.init_node(NODE_NAME)

    port = rospy.get_param('~port', 8008)

    pkg_path = get_package_path('lg_common')
    share_path = os.path.abspath(os.path.join(pkg_path, os.pardir))
    logger.info('Serving files from {} on port {}'.format(share_path, port))

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
