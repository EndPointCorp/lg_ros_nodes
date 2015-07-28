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
    nlc_timeout = rospy.get_param('~nlc_timeout', 10)
    
    kmlsync_server = tornado.web.Application([
        (r'/master.kml', KmlMasterHandler),
        (r'/network_link_update.kml', KmlUpdateHandler, dict(nlc_timeout=nlc_timeout)),
        (r'/query.html', KmlQueryHandler),
    ], debug=True)

    rospy.wait_for_service('/kmlsync/state')
    rospy.wait_for_service('/kmlsync/playtour_query')

    kml_state = KmlState()
    kmlsync_server.playtour = PlaytourQuery()
    kmlsync_server.asset_service = rospy.ServiceProxy('/kmlsync/state', kml_state, persistent=True)
    kmlsync_server.playtour_service = rospy.ServiceProxy('/kmlsync/playtour_query', kmlsync_server.playtour, persistent=True)
    
    kmlsync_server.listen(port)
    ros_tornado_spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
