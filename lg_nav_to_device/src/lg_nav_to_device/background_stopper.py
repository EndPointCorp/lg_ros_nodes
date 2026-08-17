import rospy
import threading
import types
from lg_common.helpers import load_director_message, find_window_with_activity
from lg_msg_defs.msg import ApplicationState


class BackgroundStopper:
    def __init__(self, disable_activities, device_writer):
        self.disable_activities = disable_activities
        self.device_writer = device_writer
        # Preserve DeviceWriter's enabled startup state until Earth reports its
        # first state, then include that state in every routing decision.
        self._earth_visible = True
        self._current_scene_slug = ''
        self._disabled_scene_slug = ''
        self._activity_disabled = False
        self._slug_disabled = False
        self._lock = threading.Lock()
        self._states = {}

    def _set_writer_state(self, state):
        self.device_writer.state = state

    def _recompute_writer_state(self):
        # No callback may independently enable navigation.  It is enabled only
        # when every known routing condition permits it.
        self._set_writer_state(
            self._earth_visible and
            not self._activity_disabled and
            not self._slug_disabled and
            not any(self._states.values())
        )

    def handle_earth_state(self, msg):
        with self._lock:
            self._earth_visible = msg.state == ApplicationState.VISIBLE
            self._recompute_writer_state()

    def handle_scene(self, msg):
        with self._lock:
            self._handle_scene(msg)

    def _handle_scene(self, msg):
        self._states = {}

        data = load_director_message(msg)

        self._current_scene_slug = data.get('slug')
        self._slug_disabled = bool(
            self._current_scene_slug and
            self._disabled_scene_slug and
            self._current_scene_slug == self._disabled_scene_slug
        )

        self._activity_disabled = False
        for activity in self.disable_activities:
            window = find_window_with_activity(data, activity)
            if len(window) > 0:
                self._activity_disabled = True
                break

        self._recompute_writer_state()

    def handle_slug(self, msg):
        with self._lock:
            self._handle_disable_for_scene_slug(msg)

    def _handle_disable_for_scene_slug(self, msg):
        slug_req = msg.data

        self._disabled_scene_slug = slug_req
        self._slug_disabled = bool(
            self._current_scene_slug and
            self._disabled_scene_slug and
            self._current_scene_slug == self._disabled_scene_slug
        )
        self._recompute_writer_state()

    def handle_disabled_state(self, topic, msg):
        with self._lock:
            self._handle_disabled_state(topic, msg)

    def _handle_disabled_state(self, topic, msg):
        self._states[topic] = msg.state == ApplicationState.VISIBLE
        self._recompute_writer_state()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
