#!/usr/bin/env python3

PKG = 'lg_keyboard'
NAME = 'test_onboard_router'

import time
import unittest

import rospy

from lg_msg_defs.msg import StringArray
from std_msgs.msg import Bool
from interactivespaces_msgs.msg import GenericMessage
from lg_common.test_helpers import gen_browser_window
from lg_common.test_helpers import gen_touch_window
from lg_common.test_helpers import gen_scene
from lg_common.test_helpers import gen_scene_msg
from lg_common.test_helpers import wait_for_assert_ge


class TestOnboardRouterOnline(unittest.TestCase):
    def setUp(self):
        self.activates = []
        self.activates_sub = rospy.Subscriber(
            '/lg_onboard/activate',
            StringArray,
            self.activates.append
        )
        self.director_publisher = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
        self.visibility_publisher = rospy.Publisher('/lg_onboard/visibility', Bool, queue_size=10)
        rospy.sleep(1)

    def tearDown(self):
        self.activates_sub.unregister()
        time.sleep(1)

    def test_1_sending_messages_work(self):
        msg = GenericMessage(type='json', message='{}')
        self.director_publisher.publish(msg)
        time.sleep(1)
        self.visibility_publisher.publish(Bool(data=True))
        time.sleep(1)
        wait_for_assert_ge(lambda: len(self.activates), 1, 30)
        self.assertEqual('kiosk', self.activates[-1].strings[0])

    def test_2_default_viewport_no_route_touch(self):
        """
        Generate message that will contain a browser
        without route touch set to `true`.

        Default viewport should be emitted after visibility message
        on the activate topic

        """
        window = gen_browser_window(route=False, target='cthulhu_fhtagn')
        scene = gen_scene([window])
        scene_msg = gen_scene_msg(scene)
        self.director_publisher.publish(scene_msg)
        time.sleep(1)
        # need to ensure visibility last value flip
        self.visibility_publisher.publish(Bool(data=False))
        self.visibility_publisher.publish(Bool(data=True))
        time.sleep(3)
        self.assertEqual('kiosk', self.activates[-1].strings[0])

    def test_3_default_viewport_no_route_touch(self):
        """
        Generate message that will contain two browsers
        without route touch set to `true`.

        Default viewport should be emitted after visibility message
        on the activate topic.

        """
        window1 = gen_browser_window(route=False, target='cthulhu_fhtagn')
        window2 = gen_browser_window(route=False, target='iah_iah')
        scene = gen_scene([window1, window2])
        scene_msg = gen_scene_msg(scene)
        self.director_publisher.publish(scene_msg)
        time.sleep(1)
        # need to ensure visibility last value flip
        self.visibility_publisher.publish(Bool(data=False))
        self.visibility_publisher.publish(Bool(data=True))
        time.sleep(1)
        self.assertEqual('kiosk', self.activates[-1].strings[0])

    def test_4_route_touch_on_one_viewport(self):
        """
        Generate message that will contain two browsers
        without with route touch set to `true` on one of them

        The cthulhu_fhtagn viepwort should be emitted

        """
        window1 = gen_touch_window(
            route=True,
            target='blaaah',
            source='cthulhu_fhtagn',
            activity='mirror')
        window2 = gen_touch_window(
            route=False,
            source='iah_iah',
            target='asfdnewrq',
            activity='mirror')
        scene = gen_scene([window1, window2])
        scene_msg = gen_scene_msg(scene)
        self.director_publisher.publish(scene_msg)
        time.sleep(1)
        # need to ensure visibility last value flip
        self.visibility_publisher.publish(Bool(data=False))
        self.visibility_publisher.publish(Bool(data=True))
        time.sleep(1)
        self.assertEqual('cthulhu_fhtagn', self.activates[-1].strings[0])

    def test_5_route_touch_on_two_viewports(self):
        """
        Generate message that will contain two browsers
        without with route touch set to `true` on one of them

        The cthulhu_fhtagn viepwort should be emitted

        """
        window1 = gen_touch_window(
            route=True,
            source='cthulhu_fhtagn',
            target='123rtghj',
            activity='mirror')
        window2 = gen_touch_window(
            route=True,
            source='iah_iah',
            target='123rtghj',
            activity='mirror')
        scene = gen_scene([window1, window2])
        scene_msg = gen_scene_msg(scene)
        self.director_publisher.publish(scene_msg)
        time.sleep(1)
        # need to ensure visibility last value flip
        self.visibility_publisher.publish(Bool(data=False))
        self.visibility_publisher.publish(Bool(data=True))
        time.sleep(1)
        self.assertTrue('cthulhu_fhtagn' in self.activates[-1].strings)
        self.assertTrue('iah_iah' in self.activates[-1].strings)


if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME)
    time.sleep(3)
    rostest.rosrun(PKG, NAME, TestOnboardRouterOnline)
