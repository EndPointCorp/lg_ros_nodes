#!/usr/bin/env python
import rospy
from lg_earth import KmlSyncState
from lg_common.helpers import get_params
from interactivespaces_msgs.msg import GenericMessage
from lg_earth.srv import KmlState, PlaytourQuery, PlanetQuery
from lg_common.helpers import make_soft_relaunch_callback


def main():
    rospy.init_node('kml_service', anonymous=True)

    topic = get_params('~director_topic', '/director/scene')
    service_channel = get_params('~service_channel', 'kmlsync/state')
    playtour_channel = get_params('~playtour_channel', 'kmlsync/playtour_query')
    planet_channel = get_params('~planet_channel', 'kmlsync/planet_query')
    s = KmlSyncState()
    rospy.Subscriber(topic, GenericMessage, s._save_state)
    rospy.Service(service_channel, KmlState, s._process_service_request)
    rospy.Service(playtour_channel, PlaytourQuery, s._send_playtour_query)
    rospy.Service(planet_channel, PlanetQuery, s._send_planet_query)
    make_soft_relaunch_callback(s._handle_soft_relaunch, groups=["earth"])

    rospy.spin()

if __name__ == '__main__':
    main()
