#!/usr/bin/env python
import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_earth.srv import KmlState


class OurState:
    def __init__(self, topic):
        self.sub = rospy.Subscriber(topic, GenericMessage, self._save_state)
        self.srv = rospy.Service('kmlsync/state', KmlState, self._process_service_request)
        self.state = None

    def _save_state(self, msg):
        self.state = KmlState()
        self.state.type = msg.type
        self.state.message = msg.message

    def _process_service_request(self, req):
        return { 'type': self.state.type, 'message': self.state.message}
