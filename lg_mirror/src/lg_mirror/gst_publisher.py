import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

import rospy
from sensor_msgs.msg import CompressedImage


class PipelineException(Exception):
    pass


class VideoFormatException(Exception):
    pass


class GstPublisher(threading.Thread):
    def __init__(self, pipeline, image_pub):
        super(GstPublisher, self).__init__()
        self.image_pub = image_pub

        self.pipeline = Gst.parse_launch(pipeline)
        if self.pipeline is None:
            raise PipelineException(
                'Failed to parse pipeline: {}'.format(pipeline)
            )

        self.sink = self.pipeline.get_by_name('sink')
        if self.sink is None:
            raise PipelineException(
                'Could not find an element named "sink" in pipeline: {}'.format(pipeline)
            )

        self.image_msg = CompressedImage()

    @staticmethod
    def get_encoding_stride(fmt):
        # only useful when publishing raw images
        encoding = None
        stride = None

        if fmt == 'UYVY':
            encoding = 'yuv422'
            stride = 2
        elif fmt == 'RGB':
            encoding = 'rgb8'
            stride = 3
        elif fmt == 'RGBA':
            encoding = 'rgba8'
            stride = 4
        elif fmt == 'RGB16':
            encoding = 'rgb16'
            stride = 6
        elif fmt == 'BGR':
            encoding = 'bgr8'
            stride = 3
        elif fmt == 'BGRA':
            encoding = 'bgra8'
            stride = 4
        elif fmt == 'BGR16':
            encoding = 'bgr16'
            stride = 6
        elif fmt == 'GRAY8':
            encoding = 'mono8'
            stride = 1

        if encoding is None or stride is None:
            raise VideoFormatException('Format not supported: {}'.format(fmt))

        return encoding, stride

    @staticmethod
    def parse_caps(caps):
        # only useful when publishing raw images
        width = None
        height = None
        fmt = None
        num_caps_structs = caps.get_size()
        for i in range(0, num_caps_structs):
            cs = caps.get_structure(i)
            width = cs.get_value('width')
            height = cs.get_value('height')
            fmt = cs.get_value('format')

        assert width is not None
        assert height is not None
        assert fmt is not None

        return width, height, fmt

    def _publish_sample(self):
        sample = self.sink.emit('pull-sample')
        timestamp = rospy.Time.now()
        buf = sample.get_buffer()
        #caps = sample.get_caps()
        #width, height, fmt = GstPublisher.parse_caps(caps)
        # TODO(mv): get format from caps
        self.image_msg.format = 'jpeg'
        self.image_msg.header.stamp = timestamp
        self.image_msg.data = buf.extract_dup(0, buf.get_size())
        self.image_pub.publish(self.image_msg)

    def run(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        while self.pipeline.target_state is Gst.State.PLAYING and not rospy.is_shutdown():
            self._publish_sample()
        self.stop()

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)


"""
if __name__ == '__main__':
    import time
    rospy.init_node('gst_test')
    pub = rospy.Publisher('/gst_test', Image, queue_size=1)
    pipeline = 'videotestsrc is-live=true ! capsfilter caps=video/x-raw,format=UYVY,width=1920,height=1080,framerate=30/1 ! appsink name=sink'
    gst = GstPublisher(pipeline, pub)
    gst.start()
    rospy.spin()
"""
