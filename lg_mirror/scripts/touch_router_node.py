#!/usr/bin/env python

from functools import partial
import rospy
import sys

from lg_mirror.touch_router import TouchRouter
from lg_common.helpers import on_new_scene, load_director_message
from lg_common.msg import StringArray
from lg_common.helpers import handle_initial_state
from lg_mirror.touch_router import SubscribeListener


def main():
    rospy.init_node('lg_mirror_touch_router')

    default_viewport = rospy.get_param('~default_viewport', None)
    touch_router = TouchRouter(default_viewport)

    def publish_active_touch_routes(routes):
        routes_pub.publish(StringArray(routes))

    new_listener_cb = partial(touch_router.handle_new_listener, publish_active_touch_routes)

    routes_pub = rospy.Publisher(
        '/lg_mirror/active_touch_routes',
        StringArray,
        queue_size=10,
        latch=True,
        subscriber_listener=SubscribeListener(new_listener_cb)
    )

    # Hacky callback to parse the initial scene.
    def handle_initial_scene_msg(msg):
        d = load_director_message(msg)
        touch_router.handle_scene(publish_active_touch_routes, d)

    handle_initial_state(handle_initial_scene_msg)

    scene_cb = partial(touch_router.handle_scene, publish_active_touch_routes)

    on_new_scene(scene_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
