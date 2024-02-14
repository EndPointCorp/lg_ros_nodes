#!/usr/bin/env python3

from functools import partial
import rospy
import sys

from lg_mirror.touch_router import TouchRouter
from lg_common.helpers import on_new_scene, load_director_message
from lg_msg_defs.msg import EvdevEvents, RoutedEvdevEvents, StringArray
from lg_common.helpers import handle_initial_state
from lg_mirror.touch_router import SubscribeListener
from lg_msg_defs.srv import TouchRoutes
from lg_common.helpers import run_with_influx_exception_handler
from std_msgs.msg import Bool

NODE_NAME = 'lg_mirror_router'


def main():
    rospy.init_node(NODE_NAME)

    default_viewport = rospy.get_param('~default_viewport', None)
    spacenav_viewport = rospy.get_param('~spacenav_viewport', 'fake_wall_a')
    device_id = rospy.get_param('~device_id', 'default')
    non_multitouch_activities = [
        x.strip() for x in rospy.get_param('~non_multitouch_activities', 'unity').split(',')
    ]
    non_multitouch_windows = [
        x.strip() for x in rospy.get_param('~non_multitouch_windows', 'earth,cesium,streetview,panovideo,panoviewer,unity').split(',')
    ]
    event_pub = rospy.Publisher(f'/lg_mirror/{device_id}/routed_events', RoutedEvdevEvents, queue_size=100)
    router = TouchRouter(
        event_pub,
        spacenav_viewport,
        default_viewport=default_viewport,
        non_multitouch_activities=non_multitouch_activities,
        non_multitouch_windows=non_multitouch_windows,
    )
    route_topic = '/lg_mirror/{}/active_routes'.format(device_id)

    def publish_active_routes(routes):
        routes_pub.publish(StringArray(routes))

    new_listener_cb = partial(router.handle_new_listener, publish_active_routes)

    routes_pub = rospy.Publisher(
        route_topic,
        StringArray,
        queue_size=10,
        subscriber_listener=SubscribeListener(new_listener_cb)
    )

    # Hacky callback to parse the initial scene.
    def handle_initial_scene_msg(msg):
        d = load_director_message(msg)
        router.handle_scene(publish_active_routes, d)

    handle_initial_state(handle_initial_scene_msg)

    rospy.Service(route_topic, TouchRoutes, router.handle_service_request)

    scene_cb = partial(router.handle_scene, publish_active_routes)

    on_new_scene(scene_cb)

    rospy.Subscriber('/touchscreen/toggle', Bool, router.handle_touchmenu_state, queue_size=100)
    events_topic = rospy.get_param('~events_topic', 'events')
    rospy.Subscriber(f'/lg_mirror/{device_id}/{events_topic}', EvdevEvents, router.handle_touch_event, queue_size=100)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
