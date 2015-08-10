#!/usr/bin/env python
import rospy
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from lg_earth.srv import KmlState, PlaytourQueryRequest
import json


class KmlSyncState:
    def __init__(self):
        self.state = None
        self.playtour_pub = rospy.Publisher('/earth/query/tour', String, queue_size=10)

    def _save_state(self, msg):
        try:
            state = json.loads(msg.message)
            assert isinstance(state, dict)
            assert 'windows' in state
            self.state = state
        except AssertionError:
            rospy.logerr('Invalid message - keeping previous state')
        except ValueError:
            rospy.logerr("Non json value published - keeping previous state")

    def _process_service_request(self, req):
        if self.state is None:
            return {'assets': []}
        window_slug = req.window_slug
        for window in self.state['windows']:
            if 'presentation_viewport' not in window or 'assets' not in window:
                continue
            if not window['presentation_viewport'] == window_slug:
                continue
            if 'activity' not in window or window['activity'] != 'earth':
                continue
            return {'assets': window['assets']}
        # couldn't find specific window_slug inside state
        return {'assets': []}

    def _send_playtour_query(self, req):
        self.playtour_pub.publish(String(req.tourname))
        return {'response': True}
