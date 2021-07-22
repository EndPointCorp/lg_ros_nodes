#!/usr/bin/env python3

import subprocess
import threading
from collections import namedtuple

import rospy
from lg_common import ManagedWindow
from lg_msg_defs.msg import ApplicationState, MediaOverlays

StreamInfo = namedtuple('overlay', ['name', 'viewport', 'location'])

STREAMS = {
    "201": "224.42.42.1",
    "202": "224.42.42.2",
}


class RespawningProcess(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.args = args
        self.kwargs = kwargs
        self.lock = threading.Lock()
        self.daemon = True
        self.respawn = True

    def run(self):
        while True:
            with self.lock:
                if not self.respawn:
                    break
                rospy.logdebug("starting process with args: {}, kwargs: {}".format(self.args, self.kwargs))
                self.proc = subprocess.Popen(*self.args, **self.kwargs)
            self.proc.wait()

    def stop(self):
        with self.lock:
            self.respawn = False
            self.proc.terminate()
            self.proc = None


class Player(RespawningProcess):
    def __init__(self, stream_info):
        window_id = ".".join(stream_info)
        geometry = ManagedWindow.lookup_viewport_geometry(stream_info.viewport)
        self.window = ManagedWindow(
            w_name=window_id,
            geometry=geometry,
            layer=ManagedWindow.LAYER_ABOVE,
        )
        self.window.set_visibility(True)
        self.window.converge()

        cmd = f"/usr/bin/ffplay -nostats -fflags nobuffer -flags low_delay " \
              f"-framedrop -analyzeduration 3000000 -noborder -probesize 32 -alwaysontop " \
              f"-codec:v h264 -sn " \
              f"-window_title \'{window_id}\' rtp://{STREAMS[stream_info.location]}:4953" \
              .split()

        super().__init__(cmd)


class MediaLauncher(object):
    """Maintain a list of active streams based on MediaOverlays messages"""
    def __init__(self, viewports):
        self.viewports = viewports

        self.active_overlays = {}

    def handle_message(self, message):
        """Handle incoming ros messages
        Converts MediaOverlays message to list of named tuples
        Set diff against active overlays to determine streams to add/remove
        """
        message_overlays = []
        for msg in message.overlays:
            if msg.viewport in self.viewports:
                message_overlays.append(StreamInfo(msg.name, msg.viewport, msg.location))
            else:
                rospy.logdebug("Pass")

        stale_overlays = set(self.active_overlays.keys()).difference(set(message_overlays))
        new_overlays = set(message_overlays).difference(set(self.active_overlays.keys()))

        rospy.loginfo(
            f"New message. Media in the msg: {len(message_overlays)} | "
            f"Overlays to remove: {len(stale_overlays)} | "
            f"Overlays to add: {len(new_overlays)}"
        )
        rospy.logdebug(f"Stale overlays \n\t{stale_overlays}\nNew overlays\n\t{new_overlays}")

        for item in stale_overlays:
            rospy.logdebug(f"Removing overlay from active \n\t{item}")
            stale_overlay = self.active_overlays.pop(item)
            stale_overlay.stop()

        for item in new_overlays:
            rospy.logdebug(f"Adding overlay to active \n\t{item}")
            self.active_overlays[item] = self._make_player(item)

        rospy.loginfo(
            f"Finshed message handling. | "
            f"Active overlays: {len(self.active_overlays)}"
        )

    @staticmethod
    def _make_player(stream_info):
        """Returns a new ManagedApplication for the stream message"""
        player = Player(stream_info)
        rospy.logdebug(f"Starting new Player for {stream_info}")
        player.start()

        return player


def main():
    rospy.init_node('media_launcher', log_level=rospy.DEBUG)

    viewports = [param.strip() for param in rospy.get_param('~viewports', '').split(',')]
    launcher = MediaLauncher(viewports)
    rospy.Subscriber('/media_overlays', MediaOverlays, launcher.handle_message)

    rospy.loginfo(f"Started media_launcher node on topic /media_overlays for viewports {viewports}")

    rospy.spin()


if __name__ == '__main__':
    main()
