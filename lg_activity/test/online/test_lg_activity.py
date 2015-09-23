#!/usr/bin/env python

PKG = 'lg_activity'
NAME = 'test_lg_activity'

import rospy
import unittest


# ActivitySource - a class that wraps source
# ActivitySourceDetector returns list of ActivitySources
# ActivityTracker subscribes to topics from ActivitySources and runs ActivitySources checks on them

from lg_activity import ActivitySource
from lg_activity import ActivityTracker
from lg_activity import ActivitySourceDetector
from lg_common.helpers import unpack_activity_sources, build_source_string


ACTIVITY_TRACKER_PARAM = '/spacenav/twist:geometry_msgs/Twist:delta'

class MockPublisher:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg)


class SpaceNavMockSource:
    source = {
        "topic": "/spacenav/twist",
        "message_type": "geometry_msgs/Twist",
        "strategy": "delta",
        "slot": None,
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
        "slot": None,
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
        self.callback = foo_cb
        self.strategy = 'delta'
        self.activity_source_spacenav_delta = ActivitySource(topic=self.topic,
                                                             message_type=self.message_type,
                                                             callback=self.callback,
                                                             strategy=self.strategy)

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

    def test_spacenav_in_range(self):
        spacenav = SpaceNavMockSource()
        s = spacenav.source
        activity_source = ActivitySource(
            topic=s['topic'], message_type=s['message_type'], strategy=s['strategy'],
            slot=s['slot'], value_min = s['value_min'], value_max=s['value_max'], callback=foo_cb)


def foo_cb(msg):
    """Do nothing callback"""
    pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestActivityTracker)
