#!/usr/bin/env python

import rospy
import tornado.ioloop
import tornado.web

def debug():
    import pdb
    pdb.set_trace()

class ParamHandler(tornado.web.RequestHandler):
    def _parse_path(self):
        return self.request.uri.split('?')[0]

    def get(self, *args, **kwargs):
        ros_param = self._parse_path()
        p = rospy.get_param(ros_param)
        if p:
            self.write(p)
            return


def main():
    rospy.init_node("param_server")
    port = int(rospy.get_param("port", 9812))
    application = tornado.web.Application([
        (r"/(.*)", ParamHandler),
    ])

    application.listen(port)
    ioloop = tornado.ioloop.IOLoop.current()
    rospy.on_shutdown(ioloop.stop)
    ioloop.start()


if __name__ == '__main__':
    main()
