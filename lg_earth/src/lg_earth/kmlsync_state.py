#!/usr/bin/env python3
import rospy
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from lg_msg_defs.srv import KmlStateResponse, PlaytourQueryResponse, PlanetQueryResponse
import json
from lg_common.logger import get_logger
from .scene_assets import assets_for_renderer, is_base_only_scene
logger = get_logger('kml_sync_state')


class KmlSyncState:
    def __init__(self):
        self.state = None
        self.playtour_pub = rospy.Publisher('/earth/query/tour', String, queue_size=10)
        self.planet_pub = rospy.Publisher('/earth/query/planet', String, queue_size=10)

    def _save_state(self, msg):
        try:
            state = json.loads(msg.message)
            assert isinstance(state, dict)
            assert 'windows' in state
            # A base-only compatibility scene changes visibility, not content.
            # Retaining the last content scene lets /base/select (and old
            # open-* scenes) switch renderers without unloading KML/KMZ.
            if is_base_only_scene(state):
                return
            self.state = state
        except AssertionError:
            logger.warning('Invalid message - keeping previous state')
        except ValueError:
            logger.warning("Non json value published - keeping previous state")

    def _process_service_request(self, req):
        if self.state is None:
            return KmlStateResponse(assets=[])
        return KmlStateResponse(assets=assets_for_renderer(
            self.state, req.window_slug, 'earth'))

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
