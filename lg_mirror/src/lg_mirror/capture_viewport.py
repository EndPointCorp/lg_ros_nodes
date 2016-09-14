import threading

from lg_common import ManagedWindow
from lg_common.helpers import load_director_message
from lg_common.msg import WindowGeometry
from appctl_support import ProcController
from lg_mirror.constants import MIRROR_ACTIVITY_TYPE
from lg_mirror.utils import viewport_to_multicast_group
from lg_mirror.utils import get_mirror_port, aspect_scale_source


CAPTURE_CMD = 'gst-launch-1.0'
CAPTURE_ARGS = [
    'ximagesrc',
    'startx={startx}', 'starty={starty}', 'endx={endx}', 'endy={endy}',
    'display-name={display}', 'show-pointer={show_pointer}',
    'use-damage=false',
    '!',
    'videoscale',
    '!',
    'videoconvert',
    '!',
    'capsfilter',
    'caps=video/x-raw,format=I420,width={target_width},height={target_height},framerate=60/1',
    '!',
    'queue',
    '!',
    'jpegenc', 'quality={quality}',
    '!',
    'queue',
    '!',
    'rtpjpegpay',
    '!',
    'udpsink', 'host={addr}', 'port={port}', 'sync=false'
]


class CaptureViewport:
    def __init__(self, viewport, display, quality, show_pointer, host=None, port=None):
        self.viewport = str(viewport)
        self.display = str(display)
        self.quality = int(quality)
        self.show_pointer = show_pointer
        if host is not None:
            self.addr = str(host)
        else:
            self.addr = viewport_to_multicast_group(viewport)
        if port is not None:
            self.port = int(port)
        else:
            self.port = get_mirror_port()
        self.proc = None
        self.lock = threading.Lock()

        self.geometry = ManagedWindow.lookup_viewport_geometry(self.viewport)
        self._previous_width = None
        self._previous_height = None

    def _start_capture(self, window):
        # Find dimensions that fit in the destination window,
        # while preserving the source aspect ratio.
        target_geometry = WindowGeometry(
            x=window['x_coord'],
            y=window['y_coord'],
            width=window['width'],
            height=window['height']
        )
        target_width, target_height = aspect_scale_source(
            target_geometry,
            self.geometry
        )

        # If we were already capturing for this target, NOP.
        if (self.proc is not None
                and self._previous_width == target_width
                and self._previous_height == target_height):
            return
        # Otherwise, continue exclusively.
        self._end_capture()

        self._previous_width = target_width
        self._previous_height = target_height

        args = map(lambda arg: arg.format(
            startx=self.geometry.x,
            starty=self.geometry.y,
            # Subtract 1 from width/height to workaround gstreamer wierdness.
            endx=self.geometry.x + self.geometry.width - 1,
            endy=self.geometry.y + self.geometry.height - 1,
            show_pointer=str(self.show_pointer).lower(),
            quality=self.quality,
            target_width=target_width,
            target_height=target_height,
            addr=self.addr,
            port=self.port,
            display=self.display
        ), CAPTURE_ARGS)

        cmd = [CAPTURE_CMD]
        cmd.extend(args)

        self.proc = ProcController(cmd)
        self.proc.start()

    def _end_capture(self):
        if self.proc is not None:
            self.proc.stop()
        self._previous_width = None
        self._previous_height = None

    def _handle_scene_msg(self, msg):
        scene = load_director_message(msg)

        windows = scene.get('windows', [])
        for window in windows:
            activity = window.get('activity', '')
            if activity != MIRROR_ACTIVITY_TYPE:
                continue
            config = window.get('activity_config', {})
            source_viewport = config.get('viewport', '')
            # allow viewport:// specifier
            source_viewport = source_viewport.replace('viewport://', '')
            if source_viewport == self.viewport:
                self._start_capture(window)
                return

        # No matching assets, end capture.
        self._end_capture()

    def handle_scene_msg(self, msg):
        """
        Handles a raw scene message.

        Args:
            msg (GenericMessage)
        """
        with self.lock:
            self._handle_scene_msg(msg)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
