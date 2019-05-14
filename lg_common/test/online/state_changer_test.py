#!/usr/bin/env python

PKG = 'lg_common'
NAME = 'test_state_changer'

import time
import rospy
import unittest

from lg_common.msg import StringArray
from lg_common.msg import ApplicationState
from lg_common import StateChanger

LPNODE = NAME
SUB = 0
MOCK = 1


class MockSub:
    def __init__(self):
        self.state = []

    def cb(self, msg):
        self.state.append(msg.state)


class TestStateChanger(unittest.TestCase):
    def setUp(self):
        self.subs = []

        mock = MockSub()
        sub = rospy.Subscriber('/a/state', ApplicationState, mock.cb)
        self.subs.append([sub, mock])
        mock = MockSub()
        sub = rospy.Subscriber('/b/state', ApplicationState, mock.cb)
        self.subs.append([sub, mock])
        mock = MockSub()
        sub = rospy.Subscriber('/c/state', ApplicationState, mock.cb)
        self.subs.append([sub, mock])
        self.pub = rospy.Publisher('/state_handler/activate', StringArray, queue_size=10)

        self.set_all_state_as(ApplicationState.HIDDEN)

    def set_all_state_as(self, state):
        ps = []
        for sub in self.subs:
            self.wait_for_sub(sub[SUB].name)
            p = rospy.Publisher(sub[SUB].name, ApplicationState, queue_size=2)
            self.wait_for_pub(p.name)
            ps.append(p)
        # give the subscribers time to finish
        rospy.sleep(1)
        for p in ps:
            p.publish(state)
        rospy.sleep(1)

    def tearDown(self):
        pass

    def test_all_start_as_hidden(self):
        """
        This makes sure all states are hidden, which should be the case
        when each test is run
        """
        for i in range(len(self.subs)):
            self.check_state(i, ApplicationState.HIDDEN)

    def check_state(self, index, state):
        """
        This asserts that the state at index matches the state passed
        """
        self.assertEqual(self.subs[index][MOCK].state[-1], state)

    def check_state_length(self, length):
        length += 1  # add one to length because setUp method publishes to state once
        for sub in self.subs:
            self.assertEqual(len(sub[MOCK].state), length)

    def test_one_active_starting_hidden(self):
        self.set_first_and_check()
        self.check_state_length(1)

    def set_first_and_check(self):
        active = [self.get_name_for_index(0)]
        self.assertEqual(active, ['/a/state'])
        self.publish_active(active)
        self.check_state(0, ApplicationState.VISIBLE)
        for i in range(1, len(self.subs)):
            self.check_state(i, ApplicationState.HIDDEN)

    def test_one_active_starting_visible(self):
        self.test_setting_all_as_visible()
        self.check_state_length(1)
        self.set_first_and_check()
        self.check_state_length(2)

    def test_setting_all_as_visible(self):
        active = [self.get_name_for_index(i) for i in range(len(self.subs))]
        self.publish_active(active)
        for i in range(len(self.subs)):
            self.check_state(i, ApplicationState.VISIBLE)

    def test_two_active(self):
        active = [self.get_name_for_index(i) for i in range(2)]
        self.publish_active(active)
        self.check_state_length(1)
        for i in range(2):
            self.check_state(i, ApplicationState.VISIBLE)
        for i in range(2, len(self.subs)):
            self.check_state(i, ApplicationState.HIDDEN)

    def test_active_not_found(self):
        active = ['/foooooo/bar']
        self.publish_active(active)
        self.check_state_length(1)
        self.test_all_start_as_hidden()

    def wait_for_sub(self, pubtopic):
        # wait at most 5 seconds for listenerpublisher to be registered
        timeout_t = time.time() + 5.0
        while not rostest.is_subscriber(
                rospy.resolve_name(pubtopic),
                rospy.resolve_name(LPNODE)) and time.time() < timeout_t:
            time.sleep(0.1)

        self.assert_(rostest.is_subscriber(
            rospy.resolve_name(pubtopic),
            rospy.resolve_name(LPNODE)), "%s is not up" % LPNODE)

    def wait_for_pub(self, pubtopic):
        # wait at most 5 seconds for listenerpublisher to be registered
        timeout_t = time.time() + 5.0
        while not rostest.is_publisher(
                rospy.resolve_name(pubtopic),
                rospy.resolve_name(LPNODE)) and time.time() < timeout_t:
            time.sleep(0.1)

        self.assert_(rostest.is_publisher(
            rospy.resolve_name(pubtopic),
            rospy.resolve_name(LPNODE)), "%s is not up" % LPNODE)

    def publish_active(self, arr):
        msg = StringArray(arr)
        self.pub.publish(msg)
        rospy.sleep(2)

    def get_name_for_index(self, index):
        return self.subs[index][SUB].name


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestStateChanger)
