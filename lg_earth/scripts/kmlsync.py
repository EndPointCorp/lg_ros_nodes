#!/usr/bin/env python

import rospy

from lg_earth import KmlMasterHandler, KmlUpdateHandler, KmlQueryHandler
from lg_earth.srv import KmlState, PlaytourQuery
from lg_common.webapp import ros_tornado_spin
import tornado.web
import tornado.ioloop


def main():
    rospy.init_node('kmlsync_server')
    port = rospy.get_param('~port', 8765)

    kmlsync_server = tornado.web.Application([
        (r'/master.kml', KmlMasterHandler),
        (r'/network_link_update.kml', KmlUpdateHandler),
        (r'/query.html', KmlQueryHandler),
    ], debug=True)

    kml_state = KmlState()
    kmlsync_server.playtour = PlaytourQuery()
    kmlsync_server.asset_service = rospy.ServiceProxy('/kmlsync/state', kml_state)
    kmlsync_server.playtour_service = rospy.ServiceProxy('/kmlsync/playtour_query', kmlsync_server.playtour)

    kmlsync_server.listen(port)
    ros_tornado_spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
