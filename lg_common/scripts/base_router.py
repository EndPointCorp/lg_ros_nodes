#!/usr/bin/env python3

import json

import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.msg import ApplicationState
from std_msgs.msg import String

from lg_common.base_router import BaseRouter, DEFAULT_BASE_ACTIVITIES
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'base_router'


def _activity_map():
    return {
        base: set(rospy.get_param('~{}_activities'.format(base), sorted(defaults)))
        for base, defaults in DEFAULT_BASE_ACTIVITIES.items()
    }


def main():
    rospy.init_node(NODE_NAME)

    selected_pub = rospy.Publisher('/base/selected', String, latch=True, queue_size=1)
    state_pubs = {
        base: rospy.Publisher('/{}/state'.format(base), ApplicationState,
                              latch=True, queue_size=1)
        for base in DEFAULT_BASE_ACTIVITIES
    }

    def publish_selection(selected):
        rospy.loginfo('Selected base application: %s', selected)
        selected_pub.publish(String(selected))
        for base, publisher in state_pubs.items():
            state = (ApplicationState.VISIBLE if base == selected
                     else ApplicationState.HIDDEN)
            publisher.publish(ApplicationState(state=state))

    try:
        router = BaseRouter(
            default_base=rospy.get_param('~default_base', 'earth'),
            activities=_activity_map(),
            on_change=publish_selection,
        )
    except ValueError as exc:
        rospy.logfatal(str(exc))
        raise

    def handle_scene(message):
        try:
            router.handle_scene(json.loads(message.message))
        except (TypeError, ValueError) as exc:
            rospy.logerr('Could not route invalid Director scene: %s', exc)

    def handle_selection(message):
        try:
            router.select(message.data)
        except ValueError as exc:
            rospy.logerr(str(exc))

    rospy.Subscriber('/director/scene', GenericMessage, handle_scene)
    rospy.Subscriber('/base/select', String, handle_selection)
    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
