#!/usr/bin/env python

import os.path

import rospy
import rospkg
import tornado.web
import tornado.ioloop

if __name__ == '__main__':
    rospy.init_node('lg_dev_webserver')

    port = rospy.get_param('~port', 8008)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lg_common')
    share_path = os.path.abspath(os.path.join(pkg_path, os.pardir))
    rospy.loginfo('Serving files from {} on port {}'.format(share_path, port))

    dev_webserver = tornado.web.Application([
        (r"/(.*)", tornado.web.StaticFileHandler, {"path": share_path}),
    ])
    dev_webserver.listen(port)

    ioloop = tornado.ioloop.IOLoop.instance()
    rospy.on_shutdown(ioloop.stop)
    ioloop.start()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
