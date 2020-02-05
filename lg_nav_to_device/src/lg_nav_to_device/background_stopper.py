import rospy
import threading
import types
from lg_common.helpers import load_director_message, find_window_with_activity
from lg_msg_defs.msg import ApplicationState


class BackgroundStopper:
    def __init__(self, disable_activities, device_writer):
        self.disable_activities = disable_activities
        self.device_writer = device_writer
        self._current_scene_slug = ''
        self._disabled_scene_slug = ''
        self._activity_disabled = False
        self._slug_disabled = False
        self._lock = threading.Lock()
        self._states = {}

    def _set_writer_state(self, state):
        self.device_writer.state = state

    def _consider_scene_slugs(self):
        if self._current_scene_slug == '':
            return
        elif self._disabled_scene_slug == '':
            self._slug_disabled = False
            self._set_writer_state(True)
        elif self._current_scene_slug == self._disabled_scene_slug:
            self._slug_disabled = True
            self._set_writer_state(False)
        else:
            self._slug_disabled = False
            self._set_writer_state(True)

    def _consider_disabled_states(self):
        if self._activity_disabled or self._slug_disabled:
            return

        for topic, visible in list(self._states.items()):
            if visible:
                self._set_writer_state(False)
                return

        self._set_writer_state(True)

    def handle_scene(self, msg):
        with self._lock:
            self._handle_scene(msg)

    def _handle_scene(self, msg):
        self._states = {}

        data = load_director_message(msg)

        self._current_scene_slug = data.get('slug')
        self._consider_scene_slugs()

        for activity in self.disable_activities:
            window = find_window_with_activity(data, activity)
            if len(window) > 0:
                self._activity_disabled = True
                self._set_writer_state(False)
                return

        self._activity_disabled = False

    def handle_slug(self, msg):
        with self._lock:
            self._handle_disable_for_scene_slug(msg)

    def _handle_disable_for_scene_slug(self, msg):
        slug_req = msg.data

        self._disabled_scene_slug = slug_req
        self._consider_scene_slugs()

    def handle_disabled_state(self, topic, msg):
        with self._lock:
            self._handle_disabled_state(topic, msg)

    def _handle_disabled_state(self, topic, msg):
        self._states[topic] = msg.state == ApplicationState.VISIBLE
        self._consider_disabled_states()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
