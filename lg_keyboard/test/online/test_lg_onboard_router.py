#!/usr/bin/env python

"""
lg_keyboard_router online tests
"""


import os
import time

import pytest
import rospy
import rospkg

from lg_keyboard import ROS_NODE_NAME
from lg_common.msg import StringArray
from std_msgs.msg import Bool
from interactivespaces_msgs.msg import GenericMessage
from lg_common.test_helpers import gen_browser_window
from lg_common.test_helpers import gen_touch_window
from lg_common.test_helpers import gen_scene
from lg_common.test_helpers import gen_scene_msg


class ReceiverMock:
    def __init__(self):
        self.msg = []

    def append(self, msg):
        self.msg.append(msg)


class TestOnboardRouterOnline(object):
    def setup_method(self, method):
        rospy.init_node("lg_keyboard_onboard_router", anonymous=True)
        self.grace_delay = 3
        rospy.sleep(self.grace_delay)
        self.onboard_visibility_receiver = ReceiverMock()
        self.onboard_activate_receiver = ReceiverMock()
        self.director_receiver = ReceiverMock()
        rospy.Subscriber(
            '/lg_onboard/visibility',
            Bool,
            self.onboard_visibility_receiver.append
        )
        rospy.Subscriber(
            '/lg_onboard/activate',
            StringArray,
            self.onboard_activate_receiver.append
        )
        rospy.Subscriber(
            '/director/scene',
            GenericMessage,
            self.director_receiver.append
        )
        self.director_publisher = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
        self.visibility_publisher = rospy.Publisher('/lg_onboard/visibility', Bool, queue_size=10)

    def active_wait(self, what, how_many, timeout=10):
        for _ in range(timeout):
            rospy.sleep(1)
            if len(what) == how_many:
                break

    def test_1_sending_messages_work(self):
        return
        self.director_publisher.publish(GenericMessage(type='json', message='{}'))
        self.visibility_publisher.publish(Bool(data=True))
        self.active_wait(self.director_receiver.msg, 1)
        assert len(self.director_receiver.msg) == 1
        assert len(self.onboard_visibility_receiver.msg) == 1
        assert self.director_receiver.msg[0] == GenericMessage(type='json', message='{}')

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
        self.active_wait(self.director_receiver.msg, 1)
        assert len(self.director_receiver.msg) == 1
        self.visibility_publisher.publish(Bool(data=True))
        self.active_wait(self.onboard_visibility_receiver.msg, 1)
        assert len(self.onboard_visibility_receiver.msg) == 1
        # activate sends first empty list (to shut down onboard) and then the
        # viewport activation list for onboards
        self.active_wait(self.onboard_activate_receiver.msg, 2)
        assert len(self.onboard_activate_receiver.msg) == 2
        assert len(self.onboard_activate_receiver.msg[1].strings) == 1
        assert self.onboard_activate_receiver.msg[1].strings[0] == 'kiosk'

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
        self.active_wait(self.director_receiver.msg, 1)
        assert len(self.director_receiver.msg) == 1
        self.visibility_publisher.publish(Bool(data=True))
        self.active_wait(self.onboard_visibility_receiver.msg, 2)
        assert len(self.onboard_visibility_receiver.msg) == 1
        assert len(self.onboard_activate_receiver.msg) == 2
        assert len(self.onboard_activate_receiver.msg[1].strings) == 2
        assert self.onboard_activate_receiver.msg[1].strings[0] == 'kiosk'

    def test_4_route_touch_on_one_viewport(self):
        """
        Generate message that will contain two browsers
        without with route touch set to `true` on one of them

        The cthulhu_fhtagn viepwort should be emitted
        """
        return
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
        rospy.sleep(self.grace_delay)
        assert len(self.director_receiver.msg) == 1
        self.visibility_publisher.publish(Bool(data=True))
        rospy.sleep(self.grace_delay)
        assert len(self.onboard_visibility_receiver.msg) == 1
        assert len(self.onboard_activate_receiver.msg) == 1
        assert len(self.onboard_activate_receiver.msg[0].strings) == 1
        assert self.onboard_activate_receiver.msg[0].strings[0] == 'cthulhu_fhtagn'

    def test_5_route_touch_on_two_viewports(self):
        """
        Generate message that will contain two browsers
        without with route touch set to `true` on one of them

        The cthulhu_fhtagn viepwort should be emitted
        """
        return
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
        rospy.sleep(self.grace_delay)
        assert len(self.director_receiver.msg) == 1
        self.visibility_publisher.publish(Bool(data=True))
        rospy.sleep(self.grace_delay)
        assert len(self.onboard_visibility_receiver.msg) == 1
        assert len(self.onboard_activate_receiver.msg) == 1
        assert len(self.onboard_activate_receiver.msg[0].strings) == 2
        assert 'cthulhu_fhtagn' in self.onboard_activate_receiver.msg[0].strings
        assert 'iah_iah' in self.onboard_activate_receiver.msg[0].strings


if __name__ == "__main__":
    # pytest must provide result XML file just as rostest.rosrun would do
    test_pkg = "lg_keyboard"
    test_name = "test_lg_onboard_router"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
