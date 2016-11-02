import threading

from lg_common import ManagedWindow
from lg_common.helpers import load_director_message
from lg_common.msg import WindowGeometry
from lg_mirror.gst_publisher import GstPublisher
from lg_mirror.constants import MIRROR_ACTIVITY_TYPE
from lg_mirror.utils import aspect_scale_source


CAPTURE_PIPELINE = [
    'ximagesrc',
    'startx={startx}', 'starty={starty}', 'endx={endx}', 'endy={endy}',
    'display-name={display}', 'show-pointer={show_pointer}',
    'use-damage=false',
    '!',
    'queue',
    'leaky=downstream', 'max-size-buffers=1', 'silent=true',
    '!',
    'videoscale',
    '!',
    'queue',
    'leaky=downstream', 'max-size-buffers=1', 'silent=true',
    '!',
    'videoconvert',
    '!',
    'jpegenc',
    'quality={quality}',
    '!',
    'capsfilter',
    'caps=image/jpeg,width={target_width},height={target_height},framerate={framerate}/1',
    '!',
    'appsink',
    'name=sink'
]


class CaptureViewport:
    def __init__(self, viewport, display, show_pointer, framerate, quality, image_pub):
        self.viewport = str(viewport)
        self.display = str(display)
        self.show_pointer = show_pointer
        self.framerate = int(framerate)
        self.quality = int(quality)
        self.image_pub = image_pub

        self.geometry = ManagedWindow.lookup_viewport_geometry(self.viewport)
        self.lock = threading.Lock()

        self._gst = None
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
        if (self._gst is not None
                and self._previous_width == target_width
                and self._previous_height == target_height):
            return
        # Otherwise, continue exclusively.
        self._end_capture()

        self._previous_width = target_width
        self._previous_height = target_height

        pipeline = ' '.join(map(lambda arg: arg.format(
            startx=self.geometry.x,
            starty=self.geometry.y,
            # Subtract 1 from width/height to workaround gstreamer wierdness.
            endx=self.geometry.x + self.geometry.width - 1,
            endy=self.geometry.y + self.geometry.height - 1,
            show_pointer=str(self.show_pointer).lower(),
            framerate=self.framerate,
            quality=self.quality,
            target_width=target_width,
            target_height=target_height,
            display=self.display
        ), CAPTURE_PIPELINE))

        self._gst = GstPublisher(pipeline, self.image_pub)
        self._gst.start()

    def _end_capture(self):
        if self._gst is not None:
            self._gst.stop()
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
