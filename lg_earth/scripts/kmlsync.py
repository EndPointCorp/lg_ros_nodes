#!/usr/bin/env python

import rospy

from lg_earth import KmlMasterHandler, KmlUpdateHandler, KmlQueryHandler
from lg_earth.srv import KmlState, PlaytourQuery
from lg_common.webapp import ros_tornado_spin
import tornado.web
import tornado.ioloop
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import write_log_to_file

def main():
    rospy.init_node('kmlsync_server')
    port = rospy.get_param('~port', 8765)

    kmlsync_server = tornado.web.Application([
        (r'/master.kml', KmlMasterHandler),
        (r'/network_link_update.kml', KmlUpdateHandler),
        (r'/query.html', KmlQueryHandler),
    ], debug=True)

    rospy.wait_for_service('/kmlsync/state')
    rospy.wait_for_service('/kmlsync/playtour_query')

    topic = rospy.get_param('~director_topic', '/director/scene')
    kmlsync_server.request_timeout = float(rospy.get_param('~request_timeout', 10))
    kmlsync_server.max_queue_size = int(rospy.get_param('~max_queue_size', 10))
    rospy.Subscriber(topic, GenericMessage, KmlUpdateHandler.get_scene_msg)
    kml_state = KmlState()
    kmlsync_server.playtour = PlaytourQuery()
    kmlsync_server.asset_service = rospy.ServiceProxy('/kmlsync/state', kml_state, persistent=True)
    kmlsync_server.playtour_service = rospy.ServiceProxy('/kmlsync/playtour_query', kmlsync_server.playtour, persistent=True)
    kmlsync_server.listen(port)
    ros_tornado_spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
