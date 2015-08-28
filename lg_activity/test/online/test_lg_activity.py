#!/usr/bin/env python

PKG = 'lg_media'
NAME = 'test_lg_activity'

import rospy
import unittest


# ActivitySource - a class that wraps source
# ActivitySourceDetector returns list of ActivitySources
# ActivityTracker subscribes to topics from ActivitySources and runs ActivitySources checks on them

from lg_activity import ActivitySource
from lg_activity import ActivityTracker
from lg_activity import ActivitySourceDetector
from lg_common.helpers import unpack_activity_sources


ACTIVITY_TRACKER_PARAM = '/spacenav/twist:geometry_msgs/Twist:delta'


class SpaceNavMockSource:
    source = { "/spacenav/twist": {
                    "msg_type": "geometry_msgs/Twist",
                    "strategy": "delta"
                    }}

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
        self.callback = None
        self.strategy = 'delta'
        self.activity_source_spacenav_delta = ActivitySource(topic=self.topic,
                                                             message_type=self.message_type,
                                                             callback=self.callback,
                                                             strategy=self.strategy)

    def test_detector_instantiated(self):
        """
        Checks whether detector got instantiated with proper sources
        """
        self.AssertEqual(type(self.detector), ActivitySourceDetector)
        self.AssertEqual(type(self.detector.sources), dict)
        self.AssertEqual(self.detector.get_source('/spacenav/twist'), SpaceNavMockSource.source)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestActivityTracker)
