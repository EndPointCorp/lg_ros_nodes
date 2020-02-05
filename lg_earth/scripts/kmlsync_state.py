#!/usr/bin/env python3
import rospy
from lg_earth import KmlSyncState
from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.srv import KmlState, PlaytourQuery, PlanetQuery
from lg_common.helpers import make_soft_relaunch_callback, handle_initial_state
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'kml_state_service'


def main():
    rospy.init_node(NODE_NAME, anonymous=True)

    topic = rospy.get_param('~director_topic', '/director/scene')
    service_channel = rospy.get_param('~service_channel', 'kmlsync/state')
    playtour_channel = rospy.get_param('~playtour_channel', 'kmlsync/playtour_query')
    planet_channel = rospy.get_param('~planet_channel', 'kmlsync/planet_query')
    s = KmlSyncState()
    rospy.Subscriber(topic, GenericMessage, s._save_state)
    handle_initial_state(s._save_state)
    rospy.Service(service_channel, KmlState, s._process_service_request)
    rospy.Service(playtour_channel, PlaytourQuery, s._send_playtour_query)
    rospy.Service(planet_channel, PlanetQuery, s._send_planet_query)
    make_soft_relaunch_callback(s._handle_soft_relaunch, groups=["earth"])

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
