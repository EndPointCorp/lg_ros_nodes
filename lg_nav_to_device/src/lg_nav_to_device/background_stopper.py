import rospy
import threading
import types
from lg_common.helpers import load_director_message, find_window_with_activity


class BackgroundStopper:
    def __init__(self, disable_activities, device_writer):
        self.disable_activities = disable_activities
        self.device_writer = device_writer
        self._current_scene_slug = ''
        self._disabled_scene_slug = ''
        self._lock = threading.Lock()

    def _set_writer_state(self, state):
        self.device_writer.state = state

    def _consider_scene_slugs(self):
        if self._current_scene_slug == '':
            return
        elif self._disabled_scene_slug == '':
            self._set_writer_state(True)
        elif self._current_scene_slug == self._disabled_scene_slug:
            self._set_writer_state(False)
        else:
            self._set_writer_state(True)

    def handle_scene(self, msg):
        with self._lock:
            self._handle_scene(msg)

    def _handle_scene(self, msg):
        data = load_director_message(msg)

        self._current_scene_slug = data.get('slug')
        self._consider_scene_slugs()

        for activity in self.disable_activities:
            window = find_window_with_activity(data, activity)
            if len(window) > 0:
                self._set_writer_state(False)
                return

    def handle_slug(self, msg):
        with self._lock:
            self._handle_disable_for_scene_slug(msg)

    def _handle_disable_for_scene_slug(self, msg):
        slug_req = msg.data

        self._disabled_scene_slug = slug_req
        self._consider_scene_slugs()
