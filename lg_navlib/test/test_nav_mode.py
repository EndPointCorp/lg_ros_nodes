#!/usr/bin/env python3
PKG = 'lg_navlib'
NAME = 'test_nav_mode'

import rospy
import unittest
from lg_msg_defs.msg import ApplicationState
from std_msgs.msg import String

DELAY = 0.25  # Allow this many seconds for pubsub connections and messaging.

# We expect these states to cause a mode publication.
ACTIVE_STATES = [
    ApplicationState.VISIBLE,
]
# We expect these states to not cause a mode publication.
INERT_STATES = [
    ApplicationState.STOPPED,
    ApplicationState.SUSPENDED,
    ApplicationState.HIDDEN,
    ApplicationState.STARTED,
]


class TestNavMode(unittest.TestCase):
    def setUp(self):
        self.mode_msgs = []
        self.mode_sub = rospy.Subscriber(
            '/lg_navlib/nav_mode',
            String,
            self.mode_msgs.append,
        )
        rospy.sleep(DELAY)

    def tearDown(self):
        self.mode_sub.unregister()

    def _expect_mode(self, mode=None):
        self.assertEqual(len(self.mode_msgs), 1)
        if mode:
            self.assertEqual(self.mode_msgs[0].data, mode)
        del self.mode_msgs[:]

    def test_01_default_mode(self):
        default_mode = rospy.get_param('/nav_mode/default_mode')
        self._expect_mode(default_mode)  # Latching topic, we should already have a message.

    def test_02_modes(self):
        self._expect_mode()  # Discard latched mode.

        modes = rospy.get_param('/nav_mode/modes')
        for mode, topics in modes.items():
            topics = [t.strip() for t in topics.split(',')]
            for topic in topics:
                pub = rospy.Publisher(topic, ApplicationState, queue_size=1)
                rospy.sleep(DELAY)
                for state in ACTIVE_STATES:
                    pub.publish(ApplicationState(state=state))
                    rospy.sleep(DELAY)
                    self._expect_mode(mode)
                for state in INERT_STATES:
                    pub.publish(ApplicationState(state=state))
                    rospy.sleep(DELAY)
                    self.assertEqual(len(self.mode_msgs), 0)


if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestNavMode)
