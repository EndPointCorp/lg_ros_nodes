import threading

from appctl_support import ProcController


CAPTURE_CMD = 'gst-launch-1.0'
CAPTURE_ARGS = [
    'v4l2src',
    'device={device}',
    '!',
    'videoscale',
    '!',
    'videoflip',
    'method={flip_method}',
    '!',
    'capsfilter',
    'caps={caps}',
    '!',
    'queue',
    '!',
    'vp8enc',
    'target-bitrate={target_bitrate}',
    'deadline=33333',
    'cpu-used=16',
    'max-quantizer={max_quantizer}',
    '!',
    'rtpvp8pay',
    '!',
    'udpsink',
    'host={addr}',
    'port={port}',
    'sync=false'
]


class CaptureWebcam:
    def __init__(self, janus_host, janus_port, device, width, height, framerate, max_quantizer, target_bitrate, flip=False):
        self.janus_host = janus_host
        self.janus_port = janus_port
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.max_quantizer = max_quantizer
        self.target_bitrate = target_bitrate

        self.proc = None
        self.lock = threading.Lock()

        if flip:
            self.flip_method = 'rotate-180'
        else:
            self.flip_method = 'none'

        self.cmd = self._get_cmd()

    def _get_cmd(self):
        caps = 'video/x-raw'
        if self.width is not None:
            caps += ',width={}'.format(self.width)
        if self.height is not None:
            caps += ',height={}'.format(self.height)
        if self.framerate is not None:
            caps += ',framerate={}/1'.format(self.framerate)

        args = [arg.format(
            device=self.device,
            caps=caps,
            max_quantizer=self.max_quantizer,
            target_bitrate=self.target_bitrate,
            addr=self.janus_host,
            port=self.janus_port,
            flip_method=self.flip_method,
        ) for arg in CAPTURE_ARGS]

        cmd = [CAPTURE_CMD]
        cmd.extend(args)

        return cmd

    def start_capture(self):
        self.end_capture()
        self.proc = ProcController(self.cmd)
        self.proc.start()

    def end_capture(self):
        if self.proc is not None:
            self.proc.stop()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
