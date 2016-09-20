import json
import md5
import rospy
import threading
from lg_common import ManagedApplication, ManagedWindow
from lg_common.helpers import load_director_message
from lg_common.msg import ApplicationState, WindowGeometry
from lg_mirror import MirrorException
from lg_mirror.utils import get_mirror_port, viewport_to_multicast_group
from lg_mirror.constants import *

PLAYBACK_CMD = 'gst-launch-1.0'
PLAYBACK_ARGS = [
    'udpsrc', 'address={rtp_host}', 'port={rtp_port}',
    '!',
    'capsfilter', 'caps=application/x-rtp,payload=96',
    '!',
    'rtpjpegdepay',
    '!',
    'jpegdec',
    '!',
    'xvimagesink', 'display={display}', 'sync=false'
]


class MirrorPlayback:
    def __init__(self, instance_name, display, source_host, source_port, geometry):
        self.instance_name = instance_name
        self.display = display
        self.source_host = source_host
        self.source_port = source_port
        self.geometry = geometry
        self.app = None
        self.playing = False

    @property
    def window_name_match(self):
        return PLAYBACK_CMD

    def _gen_command(self):
        args = map(lambda arg: arg.format(
            rtp_host=self.source_host,
            rtp_port=self.source_port,
            display=self.display
        ), PLAYBACK_ARGS)

        cmd = [PLAYBACK_CMD]
        cmd.extend(args)
        return cmd

    def _gen_window(self):
        return ManagedWindow(w_name=self.window_name_match, geometry=self.geometry)

    def start(self):
        if self.playing:
            raise MirrorException('Playback has already been started')

        cmd = self._gen_command()
        window = self._gen_window()

        self.app = ManagedApplication(cmd=cmd, window=window)
        self.app.set_state(ApplicationState.VISIBLE)
        self.playing = True

    def stop(self):
        if not self.playing:
            raise MirrorException('Playback has already been stopped')

        self.app.set_state(ApplicationState.STOPPED)
        self.app = None
        self.playing = False


class MirrorPlaybackPool:
    def __init__(self, display, viewport):
        self.display = display
        self.viewport = viewport
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
        conf_hash = md5.new(conf_json).hexdigest()[8:]
        target_viewport = conf.get('viewport', '')
        target_viewport = target_viewport.replace('viewport://', '')
        n = '_'.join(map(str, [
            self.viewport,
            target_viewport,
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
        target_viewport = conf.get('viewport', '')
        target_viewport = target_viewport.replace('viewport://', '')
        source_host = viewport_to_multicast_group(target_viewport)
        source_port = get_mirror_port()
        rospy.loginfo("Using geometry for playback: %s" % geometry)
        player = MirrorPlayback(
            instance_name=instance_name,
            display=self.display,
            source_host=source_host,
            source_port=source_port,
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
            if instance_name in previous_players.keys():
                self.players[instance_name] = previous_players[instance_name]
                del previous_players[instance_name]
                continue
            player = self._instantiate_player(window, instance_name)
            self.players[instance_name] = player
            player.start()

        # Any remaining previous players must be stopped.
        for p in previous_players.values():
            p.stop()

    def handle_scene_msg(self, msg):
        with self.lock:
            self._handle_scene_msg(msg)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
