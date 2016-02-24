#!/usr/bin/env python

import rospy

from lg_earth import KmlMasterHandler, KmlUpdateHandler, KmlQueryHandler
from lg_earth.srv import KmlState, PlaytourQuery, PlanetQuery
from lg_common.webapp import ros_tornado_spin
import tornado.web
import tornado.ioloop
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import write_log_to_file


def main():
    rospy.init_node('kmlsync_server')
    port = rospy.get_param('~port', 8765)

    KmlUpdateHandler.timeout = float(rospy.get_param('~request_timeout', 0))
    if KmlUpdateHandler.timeout < 0:
        rospy.logerror('Request timeout must be >= 0')
        return
    elif KmlUpdateHandler.timeout > 0:
        rospy.logwarn('Request timeout (long polling) in kmlsync is experimental')

    kmlsync_server = tornado.web.Application([
        (r'/master.kml', KmlMasterHandler),
        (r'/network_link_update.kml', KmlUpdateHandler),
        (r'/query.html', KmlQueryHandler),
    ], debug=True)

    rospy.wait_for_service('/kmlsync/state')
    rospy.wait_for_service('/kmlsync/playtour_query')
    rospy.wait_for_service('/kmlsync/planet_query')

    director_scene_topic = rospy.get_param('~director_topic', '/director/scene')
    rospy.Subscriber(director_scene_topic, GenericMessage, KmlUpdateHandler.get_scene_msg)
    kml_state = KmlState()
    kmlsync_server.playtour = PlaytourQuery()
    kmlsync_server.asset_service = rospy.ServiceProxy('/kmlsync/state', kml_state, persistent=True)
    kmlsync_server.playtour_service = rospy.ServiceProxy('/kmlsync/playtour_query', kmlsync_server.playtour, persistent=True)
    kmlsync_server.string = "This is a string"
    kmlsync_server.planet = PlanetQuery()
    kmlsync_server.planet_service = rospy.ServiceProxy('/kmlsync/planet_query', kmlsync_server.planet, persistent=True)
    kmlsync_server.listen(port)
    ros_tornado_spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
