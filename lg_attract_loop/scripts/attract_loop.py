#!/usr/bin/env python

import rospy


from std_msgs.msg import Bool
from std_msgs.msg import String
from lg_attract_loop import AttractLoop
from interactivespaces_msgs.msg import GenericMessage


def main():
    rospy.init_node('lg_attract_loop')

    # get all ros params needed to work
    activity_topic = rospy.get_param('~activity_topic_name', '/activity/active')
    director_scene_topic_name = rospy.get_param('~director_scene_topic_name', '/director/scene')
    director_presentation_topic_name = rospy.get_param('~director_presentation_topic_name', '/director/presentation')

    stop_action = rospy.get_param('~stop_action', 'go_blank')
    default_presentation = rospy.get_param('~default_presentation', None)

    # initialize Director publisher
    earth_query_publisher = rospy.Publisher('/earth/query/tour', String, queue_size=1)

    director_scene_publisher = rospy.Publisher(director_scene_topic_name, GenericMessage, queue_size=1)
    director_presentation_publisher = rospy.Publisher(director_presentation_topic_name, GenericMessage, queue_size=1)

    # get params

    api_url = rospy.get_param(
            '~director_api_url',
            os.getenv('DIRECTOR_API_URL', 'http://localhost:8034')
        )

    # initialize main logic class
    attract_loop = AttractLoop(api_url, director_scene_publisher, director_presentation_publisher, stop_action, earth_query_publisher, default_presentation)

    # subscribe to state changes
    rospy.Subscriber(activity_topic, Bool, attract_loop._process_activity_state_change)


    rospy.spin()

if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
