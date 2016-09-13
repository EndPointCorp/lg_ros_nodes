#!/usr/bin/env python

PKG = 'lg_mirror'
NAME = 'test_touch_router'

import os
import rospy
import unittest

from lg_mirror.constants import MIRROR_ACTIVITY_TYPE
from lg_common.msg import StringArray
from interactivespaces_msgs.msg import GenericMessage


GRACE_DELAY = 0.5  # seconds
TEST_DEFAULT_VIEWPORT = os.environ.get('TEST_VIEWPORT')
EXPECTED_DEFAULT_MSG = [] if TEST_DEFAULT_VIEWPORT is None else [TEST_DEFAULT_VIEWPORT]
TEST_MESSAGE_TEMPLATE = """
{{
  "name": "test_touch_router_name",
  "description": "test_touch_router_description",
  "resource_uri": "test_touch_router_uri",
  "slug": "test_touch_router_slug",
  "duration": 0,
  "windows": [
    {windows}
  ]
}}
"""

WINDOW_TEMPLATE = """
{{
  "activity": "{activity}",
  "activity_config": {{
    "route_touch": {route_touch},
    "viewport": "viewport://{source}"
  }},
  "assets": [
    "viewport://{source}"
  ],
  "width": 640,
  "height": 480,
  "x_coord": 0,
  "y_coord": 0,
  "presentation_viewport": "{target}"
}}
"""


def gen_window(route, source, target=TEST_DEFAULT_VIEWPORT, activity=MIRROR_ACTIVITY_TYPE):
    route_touch = 'true' if route else 'false'
    return WINDOW_TEMPLATE.format(
        activity=activity,
        source=source,
        target=target,
        route_touch=route_touch,
    )


def gen_scene(windows):
    joined_windows = ', '.join(windows)
    return TEST_MESSAGE_TEMPLATE.format(windows=joined_windows)


def gen_scene_msg(scene):
    return GenericMessage(type='json', message=scene)


class RouteReceiver:
    def __init__(self):
        self.msgs = []

    def handle_msg(self, msg):
        self.msgs.append(msg)


class TestTouchRouter(unittest.TestCase):
    def setUp(self):
        self.receiver = RouteReceiver()
        rospy.Subscriber(
            '/lg_mirror/active_touch_routes',
            StringArray,
            self.receiver.handle_msg
        )
        self.scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)

    def expect_default(self, msg):
        """Helper for when we want to check that a message is the default value."""

    def test_init_latch(self):
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(1, len(self.receiver.msgs))
        msg = self.receiver.msgs[-1]
        self.assertEqual(EXPECTED_DEFAULT_MSG, msg.strings)

    def test_no_route(self):
        window = gen_window(False, 'not_the_default')
        scene = gen_scene([window])
        scene_msg = gen_scene_msg(scene)
        self.scene_pub.publish(scene_msg)
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(2, len(self.receiver.msgs))
        msg = self.receiver.msgs[-1]
        self.assertEqual(EXPECTED_DEFAULT_MSG, msg.strings)

    def test_one_route(self):
        window0 = gen_window(True, 'not_the_default')
        window1 = gen_window(False, 'also_not_the_default')
        scene = gen_scene([window0, window1])
        scene_msg = gen_scene_msg(scene)
        self.scene_pub.publish(scene_msg)
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(2, len(self.receiver.msgs))
        msg = self.receiver.msgs[-1]
        self.assertEqual(1, len(msg.strings))
        self.assertTrue('not_the_default' in msg.strings)

    def test_two_routes(self):
        window0 = gen_window(True, 'not_the_default')
        window1 = gen_window(True, 'also_not_the_default')
        scene = gen_scene([window0, window1])
        scene_msg = gen_scene_msg(scene)
        self.scene_pub.publish(scene_msg)
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(2, len(self.receiver.msgs))
        msg = self.receiver.msgs[-1]
        self.assertEqual(2, len(msg.strings))
        self.assertTrue('not_the_default' in msg.strings)
        self.assertTrue('also_not_the_default' in msg.strings)

    def test_reset(self):
        window = gen_window(True, 'not_the_default')
        scene = gen_scene([window])
        scene_msg = gen_scene_msg(scene)
        self.scene_pub.publish(scene_msg)
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(2, len(self.receiver.msgs))
        msg = self.receiver.msgs[-1]
        self.assertEqual(1, len(msg.strings))
        self.assertTrue('not_the_default' in msg.strings)

        window = gen_window(False, 'also_not_the_default', activity='not_mirror')
        scene = gen_scene([window])
        scene_msg = gen_scene_msg(scene)
        self.scene_pub.publish(scene_msg)
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(3, len(self.receiver.msgs))
        msg = self.receiver.msgs[-1]
        self.assertEqual(EXPECTED_DEFAULT_MSG, msg.strings)


if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestTouchRouter)
