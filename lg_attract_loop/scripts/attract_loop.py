#!/usr/bin/env python

import os
import rospy


from std_msgs.msg import Bool
from std_msgs.msg import String
from lg_attract_loop import AttractLoop
from lg_attract_loop import DirectorAPIProxy
from lg_common.msg import ApplicationState
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'lg_attract_loop'


def main():
    rospy.init_node(NODE_NAME)

    # get all ros params needed to work
    activity_topic = rospy.get_param('~activity_topic_name', '/activity/active')
    director_scene_topic_name = rospy.get_param('~director_scene_topic_name', '/director/scene')
    director_presentation_topic_name = rospy.get_param('~director_presentation_topic_name', '/director/presentation')

    stop_action = rospy.get_param('~stop_action', 'go_blank_and_switch_to_planet')
    default_planet = rospy.get_param('~default_planet', 'earth')
    default_presentation = rospy.get_param('~default_presentation', None)
    default_duration = int(rospy.get_param('~default_duration', 120))

    # initialize Director publisher
    earth_query_publisher = rospy.Publisher('/earth/query/tour', String, queue_size=1)
    earth_planet_publisher = rospy.Publisher('/earth/query/planet', String, queue_size=1)
    earth_state_publisher = rospy.Publisher('/earth/state', ApplicationState, queue_size=1)

    def set_earth(*args, **kwargs):
        earth_state_publisher.publish(ApplicationState(ApplicationState.VISIBLE))

    director_scene_publisher = rospy.Publisher(director_scene_topic_name, GenericMessage, queue_size=1)
    director_presentation_publisher = rospy.Publisher(director_presentation_topic_name, GenericMessage, queue_size=1)

    # get params

    api_url = rospy.get_param(
        '~director_api_url',
        os.getenv('DIRECTOR_API_URL', 'http://localhost:8034')
    )

    # director API
    director_api_proxy = DirectorAPIProxy(api_url)

    # initialize main logic class
    attract_loop = AttractLoop(director_api_proxy,
                               director_scene_publisher,
                               director_presentation_publisher,
                               stop_action,
                               earth_query_publisher,
                               earth_planet_publisher,
                               default_presentation=default_presentation,
                               default_planet=default_planet,
                               set_earth=set_earth,
                               default_duration=default_duration)

    # subscribe to state changes
    rospy.Subscriber(activity_topic, Bool, attract_loop._process_activity_state_change)

    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
