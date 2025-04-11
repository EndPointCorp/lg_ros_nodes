#!/usr/bin/env python3

import rospy
import urllib.request, urllib.parse, urllib.error
import json
from urllib.parse import urlparse

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from lg_msg_defs.srv import USCSMessage, USCSMessageResponse, InitialUSCS, InitialUSCSResponse
from interactivespaces_msgs.msg import GenericMessage
from lg_common import USCSService
from lg_common.helpers import check_www_dependency
from lg_common.helpers import run_with_influx_exception_handler


# TODO implement this in the ros_cms side of things so
# that the initial scene can be set from the web interface
# instead of via hacky url
INITIAL_STATE = "http://lg-head:8088/director_api/initial_scene"
ON_OFFLINE_STATE = "http://lg-head:8088/director_api/on_offline_scene"
ON_ONLINE_STATE = "http://lg-head:8088/director_api/on_online_scene"
ON_INACTIVE_STATE = "http://lg-head:8088/director_api/on_inactive_scene"
ON_ACTIVE_STATE = "http://lg-head:8088/director_api/on_active_scene"
NODE_NAME = 'uscs_service'


def main():
    def set_url(param):
        url = rospy.get_param(param, '')
        if url:
            scheme = urlparse(url)
            if not scheme.port:
                port = 80
            else:
                port = scheme.port
            check_www_dependency(depend_on_scene_repository,
                                 scheme.hostname,
                                 port, param[1:],
                                 global_dependency_timeout)
        return url

    rospy.init_node(NODE_NAME, anonymous=False)

    director_topic = rospy.get_param('~director_topic', '/director/scene')
    message_topic = rospy.get_param('~message_topic', '/uscs/message')
    offline_topic = rospy.get_param('~offline_topic', '/lg_offliner/offline')
    activity_topic = rospy.get_param('~activity_topic', '/activity/active')
    depend_on_scene_repository = rospy.get_param('~depend_on_scene_repository', True)
    global_dependency_timeout = rospy.get_param('/global_dependency_timeout', 15)

    initial_state_scene_url = set_url('~initial_state_scene_url')
    on_online_state_scene_url = set_url('~on_online_state_scene_url')
    on_offline_state_scene_url = set_url('~on_offline_state_scene_url')
    on_active_state_scene_url = set_url('~on_active_state_scene_url')
    on_inactive_state_scene_url = set_url('~on_inactive_state_scene_url')

    director_scene_publisher = rospy.Publisher(
        director_topic, GenericMessage, queue_size=3)

    us = USCSService(
        initial_state_scene_url=initial_state_scene_url,
        on_online_state_scene_url=on_online_state_scene_url,
        on_offline_state_scene_url=on_offline_state_scene_url,
        on_active_state_scene_url=on_active_state_scene_url,
        on_inactive_state_scene_url=on_inactive_state_scene_url,
        director_scene_publisher=director_scene_publisher
    )

    rospy.Subscriber(director_topic, GenericMessage, us.update_uscs_message)
    rospy.Subscriber(offline_topic, Bool, us.handle_offline_message)
    rospy.Subscriber(activity_topic, Bool, us.handle_activity_message)

    rospy.Service(message_topic, USCSMessage, us.current_uscs_message)
    rospy.Service('/initial_state', InitialUSCS, us.initial_state)
    rospy.Service('/uscs/republish', Empty, us.republish)

    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
