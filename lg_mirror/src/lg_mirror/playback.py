import json
import md5
import rospy
import threading
from lg_common import ManagedBrowser, ManagedWindow
from lg_common.helpers import load_director_message
from lg_msg_defs.msg import ApplicationState, WindowGeometry
from lg_mirror import MirrorException
from lg_mirror.utils import get_mirror_port, viewport_to_multicast_group
from lg_mirror.constants import *


class MirrorPlayback:
    def __init__(self, instance_name, janus_url, source_viewport, geometry):
        self.instance_name = instance_name
        self.janus_url = janus_url
        self.source_viewport = source_viewport
        self.geometry = geometry
        self.app = None
        self.playing = False

    def start(self):
        if self.playing:
            raise MirrorException('Playback has already been started')

        url = 'http://localhost:8008/lg_mirror/webapps/playback/index.html'
        url += '?viewport=' + self.source_viewport
        url += '&janusUrl=' + self.janus_url

        self.app = ManagedBrowser(
            url=url,
            slug=self.instance_name,
            geometry=self.geometry,
        )
        self.app.set_state(ApplicationState.VISIBLE)
        self.playing = True

    def stop(self):
        if not self.playing:
            raise MirrorException('Playback has already been stopped')

        self.app.set_state(ApplicationState.STOPPED)
        self.app = None
        self.playing = False


class MirrorPlaybackPool:
    def __init__(self, display, viewport, janus_url):
        self.display = display
        self.viewport = viewport
        self.janus_url = janus_url
        self.geometry = ManagedWindow.lookup_viewport_geometry(viewport)
        self.players = {}
        self.lock = threading.Lock()

    def _extract_windows_from_scene(self, scene):
        found_windows = []

        windows = scene.get('windows', [])
        for window in windows:
            activity = window.get('activity', '')
            if activity != MIRROR_ACTIVITY_TYPE:
                continue
            viewport = window.get('presentation_viewport', '')
            if viewport == self.viewport:
                found_windows.append(window)

        return found_windows

    def _window_to_instance_name(self, window):
        conf = window.get('activity_config', {})
        conf_json = json.dumps(conf, sort_keys=True)
        conf_hash = md5.new(conf_json).hexdigest()[-6:]
        n = '_'.join(map(str, [
            self.viewport,
            window['width'],
            window['height'],
            window['x_coord'],
            window['y_coord'],
            conf_hash
        ]))
        return n

    def _instantiate_player(self, window, instance_name):
        geometry = WindowGeometry(
            width=window['width'],
            height=window['height'],
            x=self.geometry.x + window['x_coord'],
            y=self.geometry.y + window['y_coord']
        )
        conf = window.get('activity_config', {})
        source_viewport = conf.get('viewport', '')
        source_viewport = source_viewport.replace('viewport://', '')
        rospy.loginfo("Using geometry for playback: %s" % geometry)
        player = MirrorPlayback(
            instance_name=instance_name,
            janus_url=self.janus_url,
            source_viewport=source_viewport,
            geometry=geometry
        )
        return player

    def _handle_scene_msg(self, msg):
        scene = load_director_message(msg)
        windows = self._extract_windows_from_scene(scene)

        # Save the previous state for continuation and cleanup.
        previous_players = self.players
        self.players = {}

        for window in windows:
            instance_name = self._window_to_instance_name(window)
            # If we were playing this previously, promote to the new list.
            if instance_name in list(previous_players.keys()):
                self.players[instance_name] = previous_players[instance_name]
                del previous_players[instance_name]
                continue
            player = self._instantiate_player(window, instance_name)
            self.players[instance_name] = player
            player.start()

        # Any remaining previous players must be stopped.
        for p in list(previous_players.values()):
            p.stop()

    def handle_scene_msg(self, msg):
        with self.lock:
            self._handle_scene_msg(msg)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
