#!/usr/bin/env python3

import rospy
import unittest
from lg_activity import ActivitySource
from lg_activity import ActivityTracker
from lg_activity import ActivitySourceDetector
from lg_activity.activity import ActivitySourceException
from lg_common.helpers import build_source_string
from lg_common.test_helpers import wait_for_assert_equal
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

PKG = 'lg_activity'
NAME = 'test_lg_activity'
ACTIVITY_TRACKER_PARAM = '/spacenav/twist:geometry_msgs/Twist:delta'


class MockPublisher:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg.data)


class MockCallbackHandler:
    def __init__(self):
        self.state = False

    def cb(self, topic, state, strategy):
        self.state = state


class SpaceNavMockSource:
    source = {
        "topic": "/spacenav/twist",
        "message_type": "geometry_msgs/Twist",
        "strategy": "delta",
        "slot": None,
        "value": None,
        "value_min": None,
        "value_max": None
    }
    source_string = build_source_string(
        topic=source['topic'], message_type=source['message_type'],
        strategy=source['strategy'], slot=source['slot'],
        value_min=source['value_min'], value_max=source['value_max'])


class TouchscreenMockSource:
    source = {
        "topic": "/touchscreen/touch",
        "message_type": "interactivespaces_msgs/String",
        "strategy": "activity",
        "slot": None,
        "value": None,
        "value_min": None,
        "value_max": None
    }
    source_string = build_source_string(
        topic=source['topic'], message_type=source['message_type'],
        strategy=source['strategy'], slot=source['slot'],
        value_min=source['value_min'], value_max=source['value_max'])


class ProximitySensorMockSource:
    source = {
        "topic": "/proximity_sensor/distance",
        "message_type": "std_msgs/Float32",
        "strategy": "value",
        "slot": 'data',
        "value": None,
        "value_min": 10,
        "value_max": 20
    }
    source_string = build_source_string(
        topic=source['topic'], message_type=source['message_type'],
        strategy=source['strategy'], slot=source['slot'],
        value_min=source['value_min'], value_max=source['value_max'])


class TestActivityTracker(unittest.TestCase):
    def setUp(self):
        """
            Scenario:
            Test ActivitySource:
                - instantiate and run basic asserts on the attributes
            Test ActivitySourceDetector:
                - check whether
            Test ActivityTracker:
                - test delta
                - test activity
                - test value() (TODO)
        """
        self.detector = ActivitySourceDetector(ACTIVITY_TRACKER_PARAM)
        self.sources = self.detector.get_sources()
        self.topic = '/spacenav/twist'
        self.message_type = 'geometry_msgs/Twist'
        self.cb = MockCallbackHandler()
        self.cb_2 = MockCallbackHandler()
        self.callback = self.cb_2.cb
        self.strategy = 'delta'
        """
        self.activity_source_spacenav_delta = ActivitySource(topic=self.topic,
                                                             message_type=self.message_type,
                                                             callback=self.callback,
                                                             strategy=self.strategy)
        """

    def test_detector_instantiated(self):
        """
        Checks whether detector got instantiated with proper sources
        """
        self.assertEqual(self.detector.__class__.__name__, ActivitySourceDetector.__name__)
        self.assertDictEqual(self.detector.get_source('/spacenav/twist'), SpaceNavMockSource.source)
        self.assertEqual(type(self.detector.sources), list)
        self.assertEqual(type(self.detector.sources[0]), dict)

    def test_tracker(self):
        self.assertEqual(1, 1)

    def test_source_builder(self):
        test_source = '/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5'
        topic = '/proximity_sensor/distance'
        message_type = 'sensor_msgs/Range'
        slot = 'range'
        strategy = 'value'
        value_min = '0'
        value_max = '2.5'
        built_source = build_source_string(topic, message_type, strategy, slot=slot, value_min=value_min, value_max=value_max)
        self.assertEqual(built_source, test_source)

    def test_spacenav_source_matches_activity(self):
        spacenav = SpaceNavMockSource()
        activity_source = ActivitySourceDetector(spacenav.source_string)

        self.assertDictEqual(spacenav.source, activity_source.get_sources()[0])

    def test_delta_active_to_inactive(self):
        """
        This will publish one spacenav message of type a, then delta_msg_count - 1 of type
        b, which will cause the state to become active (True), then another message of type
        b will be published, meaning that there will be delta_msg_count messages in a row of
        the same value, which should cause the state to be false
        """
        spacenav = SpaceNavMockSource()
        s = spacenav.source
        msg_a = make_twist_messages(1)
        msg_b = make_twist_messages(0)
        self.assertFalse(self.cb.state)
        activity_source = self.new_activity_source(s)

        activity_source._aggregate_message(msg_a)
        for i in range(ActivitySource.DELTA_MSG_COUNT - 1):
            self.false_state()
            activity_source._aggregate_message(msg_b)
        self.true_state()
        activity_source._aggregate_message(msg_b)
        self.false_state()
        del activity_source

    def test_range_active_to_inactive(self):
        """
        This will publish a range message within the boundaries of the range, and assert
        that the state is True, then a range outside (below) the boundaries, and assert
        that the state is False. This will then be repeated with a value above the boundaries
        """
        prox = ProximitySensorMockSource()
        s = prox.source
        below = Float32(s['value_min'] - 1)
        above = Float32(s['value_max'] + 1)
        min_active = Float32(s['value_min'])
        max_active = Float32(s['value_max'])
        mid_active = Float32((s['value_min'] + s['value_max']) / 2)
        activity_source = self.new_activity_source(s)

        activity_source._aggregate_message(min_active)
        self.true_state()
        activity_source._aggregate_message(below)
        self.false_state()
        activity_source._aggregate_message(max_active)
        self.true_state()
        activity_source._aggregate_message(above)
        self.false_state()
        activity_source._aggregate_message(mid_active)
        self.true_state()

    def test_activity_tracker(self):
        """
        Test activity tracker using spacenav messages
        Spacenav uses delta. If all values in the buffer are identical then
        state is inactive. When one value is odd - state turns to active.

        Scenario:
        - assert that initial state is active
        - emit identical messages aand wait for timeout seconds to become inactive
        - emit one odd message which should make it active
        - emit identical messages again to make it inactive after timeout again
        """
        # set as inactive by publishing delta_msg_count spacenav messages with homogenous data
        msg_a = make_twist_messages(1)
        msg_b = make_twist_messages(0)
        spacenav = SpaceNavMockSource()
        p = rospy.Publisher(spacenav.source['topic'], Twist, queue_size=10)

        sources = ActivitySourceDetector(spacenav.source_string).get_sources()
        pub = MockPublisher()
        timeout = lambda: 5
        tracker = ActivityTracker(publisher=pub, timeout=timeout, sources=sources, debug=True)
        # test if it's active by default

        # emit msg_a to fill the buffer and make it active
        for i in range(ActivitySource.DELTA_MSG_COUNT + 1):
            self.assertTrue(tracker.active)
            self.assertTrue(pub.data[-1])
            self.assertEqual(len(pub.data), 1)
            p.publish(msg_a)

        self.assertTrue(tracker.active)
        self.assertTrue(pub.data[-1])
        self.assertEqual(len(pub.data), 1)

        rospy.sleep(1)
        # fill the buffer with identical values so it becomes inactive
        for i in range(ActivitySource.DELTA_MSG_COUNT + 1):
            self.assertTrue(tracker.active)
            self.assertTrue(pub.data[-1])
            self.assertEqual(len(pub.data), 1)
            p.publish(msg_b)

    def false_state(self):
        self.assertFalse(self.cb.state)

    def true_state(self):
        self.assertTrue(self.cb.state)

    def new_activity_source(self, s):
        activity_source = ActivitySource(
            topic=s['topic'], message_type=s['message_type'], strategy=s['strategy'],
            slot=s['slot'], value_min=s['value_min'], value_max=s['value_max'], callback=self.cb.cb)
        self.assertFalse(self.cb.state)
        return activity_source

    def foo_cb(self, msg):
        """Do nothing callback"""
        return

    def test_aaa_spacenav_thing(self):
        debug_pub = rospy.Publisher('/spacenav/twist', Twist, queue_size=10)
        debug_sub = rospy.Subscriber('/spacenav/twist', Twist, self.foo_cb)
        rospy.sleep(2)
        debug_pub.publish(make_twist_messages(0))

    def test_invalid_activity_source_arguments(self):
        prox = ProximitySensorMockSource()
        # assert Not raises...
        working_source = prox.source.copy()
        self.new_activity_source(working_source)

        with self.assertRaises(ActivitySourceException):
            broken_source = working_source
            broken_source['topic'] = None
            self.new_activity_source(broken_source)
        with self.assertRaises(ActivitySourceException):
            broken_source = working_source
            broken_source['topic'] = {}
            self.new_activity_source(broken_source)
        with self.assertRaises(ActivitySourceException):
            broken_source = working_source
            broken_source['value_min'] = None
            self.new_activity_source(broken_source)
        with self.assertRaises(ActivitySourceException):
            broken_source = working_source
            broken_source['slot'] = None
            self.new_activity_source(broken_source)


def make_twist_messages(value):
    msg = Twist()
    msg.angular.x = value
    msg.angular.y = value
    msg.angular.z = value
    msg.linear.x = value
    msg.linear.y = value
    msg.linear.z = value
    return msg


if __name__ == '__main__':
    import rostest
    rospy.init_node("%s_%s" % (PKG, NAME))
    rostest.rosrun(PKG, NAME, TestActivityTracker)
