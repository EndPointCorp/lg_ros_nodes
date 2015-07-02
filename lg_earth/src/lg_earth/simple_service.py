#!/usr/bin/env python
import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_earth.srv import KmlState
import json


class OurState:
    def __init__(self, topic, service_channel):
        self.topic = topic
        self.sub = rospy.Subscriber(topic, GenericMessage, self._save_state)
        self.srv = rospy.Service(service_channel, KmlState, self._process_service_request)
        self.state = None

    def _save_state(self, msg):
        try:
            self.state = json.loads(msg.message)
        except ValueError:
            rospy.logerr("Non json value published on %s, keeping previous state" % self.topic)

    def _process_service_request(self, req):
        window_slug = req.window_slug
        try:
            assert isinstance(self.state, dict)
            assert 'windows' in self.state
        except AssertionError:
            rospy.logerr("Invalid message has been stored, state might not have been set")
            return {'assets': []}
        for window in self.state['windows']:
            if 'presentation_viewport' not in window or 'assets' not in window:
                continue
            if not window['presentation_viewport'] == window_slug:
                continue
            return {'assets': window['assets']}
        # couldn't find specific window_slug inside state
        return {'assets': []}
