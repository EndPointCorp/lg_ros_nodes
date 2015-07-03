#!/usr/bin/env python
import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_earth.srv import KmlState
import json


class KmlSyncState:
    def __init__(self):
        self.state = None

    def _save_state(self, msg):
        try:
            state = json.loads(msg.message)
            assert isinstance(state, dict)
            assert 'windows' in state
            for window in state['windows']:
                if window.get('activity', '') == 'earth':
                    self.state = state
                    return
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
