#!/usr/bin/env python

from functools import partial
import rospy
import sys

from lg_mirror.touch_router import TouchRouter
from lg_common.helpers import on_new_scene
from lg_common.msg import StringArray


def main():
    rospy.init_node('lg_mirror_touch_router')

    default_viewport = rospy.get_param('~default_viewport', None)

    routes_pub = rospy.Publisher(
        '/lg_mirror/active_touch_routes',
        StringArray,
        queue_size=10,
        latch=True
    )

    def publish_active_touch_routes(routes):
        routes_pub.publish(StringArray(routes))

    touch_router = TouchRouter(default_viewport)

    # Init latching to the default viewport.
    touch_router.handle_scene(publish_active_touch_routes, {})

    scene_cb = partial(touch_router.handle_scene, publish_active_touch_routes)
    on_new_scene(scene_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
