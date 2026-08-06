#!/usr/bin/env python3
import rospy
import time
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from lg_msg_defs.srv import KmlStateResponse, PlaytourQueryResponse, PlanetQueryResponse
import json
from lg_common.logger import get_logger
logger = get_logger('kml_sync_state')


class KmlSyncState:
    def __init__(self):
        self.state = None
        # When the current scene arrived. delay_seconds/duration_seconds are
        # measured from here.
        self.scene_started = None
        self.playtour_pub = rospy.Publisher('/earth/query/tour', String, queue_size=10)
        self.planet_pub = rospy.Publisher('/earth/query/planet', String, queue_size=10)

    def _save_state(self, msg):
        try:
            state = json.loads(msg.message)
            assert isinstance(state, dict)
            assert 'windows' in state
            self.state = state
            self.scene_started = time.monotonic()
        except AssertionError:
            logger.warning('Invalid message - keeping previous state')
        except ValueError:
            logger.warning("Non json value published - keeping previous state")

    def _window_is_due(self, window):
        """
        Whether a window's assets should be loaded right now, per the
        `delay_seconds`/`duration_seconds` in its activity_config.

        Nothing is scheduled or republished: kmlsync asks us on every Earth
        poll (~1s), so the assets simply start and stop being reported. That
        also means a timed window never persists across a scene change -- the
        clock restarts with the scene, so its assets get unloaded and reloaded.
        """
        activity_config = window.get('activity_config', {}) or {}
        delay_seconds = float(activity_config.get('delay_seconds', 0) or 0)
        duration_seconds = float(activity_config.get('duration_seconds', 0) or 0)
        if delay_seconds <= 0 and duration_seconds <= 0:
            return True
        if self.scene_started is None:
            return True

        elapsed = time.monotonic() - self.scene_started
        if elapsed < delay_seconds:
            return False
        if duration_seconds > 0 and elapsed >= delay_seconds + duration_seconds:
            return False
        return True

    def _process_service_request(self, req):
        if self.state is None:
            return KmlStateResponse(assets=[])
        window_slug = req.window_slug
        # Every earth window on this viewport contributes, each filtered by its
        # own timing, so a scene can mix untimed KML with KML that comes and
        # goes on its own schedule. (This used to serve only the first matching
        # window, which made per-window timing unusable -- a second window was
        # never reached.)
        assets = []
        for window in self.state['windows']:
            if 'presentation_viewport' not in window or 'assets' not in window:
                continue
            if not window['presentation_viewport'] == window_slug:
                continue
            if 'activity' not in window or window['activity'] != 'earth':
                continue
            if not self._window_is_due(window):
                # Outside its delay/duration window: contribute nothing and let
                # kmlsync unload whatever it had.
                continue
            for asset in window['assets']:
                if asset not in assets:
                    assets.append(asset)
        return KmlStateResponse(assets=assets)

    def _send_playtour_query(self, req):
        self.playtour_pub.publish(String(req.tourname))
        return PlaytourQueryResponse(response=True)
    def _send_planet_query(self, req):
        self.planet_pub.publish(String(req.planetname))
        return PlanetQueryResponse(response=True)

    def _handle_soft_relaunch(self, msg):
        """
        On a soft relaunch it would make sense to clear the current state
        """
        self.state = None
        self.scene_started = None
