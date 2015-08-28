#!/usr/bin/env python

import rospy
from lg_common.helpers import unpack_activity_sources

class ActivitySourceNotFound(Exception):
    pass

class ActivitySourceException(Exception):
    pass

class ActivitySource:
    """
    Should be instantiated with topic, message_type, callback and strategy

    Provides:
    - subscription of a single topic with given callback
    - provides a strategy for data flowing on a topic
    - single activity source and ros callback for handling it
    """
    def __init__(self, topic=None, message_type=None, callback=None, strategy=None):
        if (not topic) or (not message_type) or (not callback) or (not strategy):
            msg = "Could not initialize ActivitySource: topic=%s, message_type=%s, callback=%s, strategy=%s" % \
                    (topic, message_type, callback, strategy)

            rospy.logerr(msg)
            raise ActivitySourceException(msg)

class ActivitySourceDetector:
    """
    Provides a getter for a dictionary of topics, messages and strategies e.g.:
        { "/spacenav/twist": {
            "msg_type": "geometry_msgs/Twist",
            "strategy": "delta"
            }
    """
    def __init__(self, sources_string):
        self.sources = unpack_activity_sources(sources_string)
        rospy.loginfo("Instantiated following activity sources: %s using provided string: %s" % (self.sources, sources_string))

    def get_sources(self):
        return self.sources

    def get_source(self, source):
        try:
            source = self.sources('source')
            return source
        except KeyError, e:
            msg = "Could not find source %s" %s
            rospy.logerr(msg)
            raise ActivitySourceNotFound(msg)

class ActivityTracker:
    """
    Provides callback for checking whether messages with applied strategy
    should trigger the activity

    Provides a topic /activity/active and emits a active: True|False message
    depending on the activity state change

    Keeps the state of activity. State should be 'active: true' by default

    Provides service for checking the activity state

    """
    def __init__(self):
        self.active = True
        pass

    def handle_message(self):
        pass
