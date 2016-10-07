#!/usr/bin/env python

"""
lg_keyboard_router online tests

starting roslaunch for development:
    roslaunch --screen lg_keyboard/test/online/test_onboard_router.test
    could do:
    py.test -s -v lg_keyboard/test/online/test_onboard_router.py

running tests manually:
    rostest lg_keyboard/test/online/test_onboard_router.test
        (as long as it contains <test> tag, it's the same as launch file)

"""


import os
import time
from multiprocessing import Array

import pytest
import rospy
import rospkg
from multiprocessing import Array

from lg_common.msg import StringArray
from std_msgs.msg import Bool
from interactivespaces_msgs.msg import GenericMessage
from lg_common.test_helpers import gen_browser_window
from lg_common.test_helpers import gen_touch_window
from lg_common.test_helpers import gen_scene
from lg_common.test_helpers import gen_scene_msg


SCENE = Array('c', 1000)  # size
VISIBILITY = Array('c', 1000)  # size
ACTIVATE = Array('c', 1000)  # size


class TestOnboardRouterOnline(object):
    def setup_method(self, method):
        rospy.Subscriber(
            '/lg_onboard/visibility',
            Bool,
            self.callback_visibility_receiver
        )
        rospy.Subscriber(
            '/lg_onboard/activate',
            StringArray,
            self.callback_activate_receiver
        )
        rospy.Subscriber(
            '/director/scene',
            GenericMessage,
            self.callback_scene_receiver
        )
        self.director_publisher = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
        self.visibility_publisher = rospy.Publisher('/lg_onboard/visibility', Bool, queue_size=10)
        SCENE.value = "UNDEFINED"
        VISIBILITY.value = "UNDEFINED"
        ACTIVATE.value = "UNDEFINED"
        # must be after pubs/subs initialization:
        rospy.init_node("lg_keyboard_onboard_router", anonymous=True)
        self.grace_delay = 3
        rospy.sleep(self.grace_delay)

    def teardown_method(self, method):
        time.sleep(1)

    @staticmethod
    def callback_scene_receiver(msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        SCENE.value = str(msg.message)

    @staticmethod
    def callback_visibility_receiver(msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        VISIBILITY.value = str(msg.data)

    @staticmethod
    def callback_activate_receiver(msg):
        rospy.loginfo("callback received type: '%s', message: %s" % (type(msg), msg))
        ACTIVATE.value = str(msg.strings)

    def active_wait(self, what, where, timeout=5):
        for _ in range(timeout):
            rospy.sleep(1)
            if where == what:
                break
        assert where == what

    def test_1_sending_messages_work(self):
        msg = GenericMessage(type='json', message='{}')
        self.director_publisher.publish(msg)
        time.sleep(1)
        self.visibility_publisher.publish(Bool(data=True))
        time.sleep(1)
        self.active_wait('{}', SCENE.value)
        self.active_wait('True', VISIBILITY.value)
        self.active_wait("['kiosk']", ACTIVATE.value)

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
        time.sleep(1)
        self.active_wait('True', VISIBILITY.value)
        self.active_wait("['kiosk']", ACTIVATE.value)

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
        self.active_wait('True', VISIBILITY.value)
        self.active_wait("['kiosk']", ACTIVATE.value)

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
        self.active_wait('True', VISIBILITY.value)
        self.active_wait("['cthulhu_fhtagn']", ACTIVATE.value)

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
        self.active_wait('True', VISIBILITY.value)
        self.active_wait("['cthulhu_fhtagn', 'iah_iah']", ACTIVATE.value)


if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    test_pkg = "lg_keyboard"
    test_name = "test_onboard_router"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
