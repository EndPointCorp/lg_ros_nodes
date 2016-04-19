#!/usr/bin/env python

import os
import rospy


from std_msgs.msg import Bool
from std_msgs.msg import String
from lg_attract_loop import AttractLoop
from lg_attract_loop import DirectorAPIProxy
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import get_params


def main():
    rospy.init_node('lg_attract_loop')

    # get all ros params needed to work
    activity_topic = get_params('~activity_topic_name', '/activity/active')
    director_scene_topic_name = get_params('~director_scene_topic_name', '/director/scene')
    director_presentation_topic_name = get_params('~director_presentation_topic_name', '/director/presentation')

    stop_action = get_params('~stop_action', 'go_blank')
    default_presentation = get_params('~default_presentation', None)

    # initialize Director publisher
    earth_query_publisher = rospy.Publisher('/earth/query/tour', String, queue_size=1)

    director_scene_publisher = rospy.Publisher(director_scene_topic_name, GenericMessage, queue_size=1)
    director_presentation_publisher = rospy.Publisher(director_presentation_topic_name, GenericMessage, queue_size=1)

    # get params

    api_url = get_params(
        '~director_api_url',
        os.getenv('DIRECTOR_API_URL', 'http://localhost:8034')
    )

    # director API
    director_api_proxy = DirectorAPIProxy(api_url)

    # initialize main logic class
    attract_loop = AttractLoop(director_api_proxy, director_scene_publisher, director_presentation_publisher, stop_action, earth_query_publisher, default_presentation)

    # subscribe to state changes
    rospy.Subscriber(activity_topic, Bool, attract_loop._process_activity_state_change)

    rospy.spin()

if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
