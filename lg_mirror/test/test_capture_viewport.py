#!/usr/bin/env python

PKG = 'lg_mirror'
NAME = 'test_capture_viewport'

import os
import psutil
import select
import socket
import struct
import unittest

import rospy
from lg_common import ManagedWindow
from appctl_support import ProcController
from interactivespaces_msgs.msg import GenericMessage
from sensor_msgs.msg import Image
from lg_mirror.utils import get_viewport_topic

CAPTURE_VIEWPORT = os.environ.get('CAPTURE_VIEWPORT')
CAPTURE_DISPLAY = os.environ.get('DISPLAY')
CAPTURE_WIDTH = int(os.environ.get('CAPTURE_WIDTH'))
CAPTURE_HEIGHT = int(os.environ.get('CAPTURE_HEIGHT'))
CAPTURE_TOPIC = get_viewport_topic(CAPTURE_VIEWPORT)

SOCK_TIMEOUT = 3  # seconds

BLANK_SCENE = GenericMessage(
    type='json',
    message='{"slug":"blank"}'
)

SCENE_TEMPLATE = """
{
  "description": "bogus",
  "duration": 0,
  "name": "test whatever",
  "resource_uri": "bogus",
  "slug": "test message",
  "windows": [
    {
      "activity": "mirror",
      "activity_config": {
        "viewport": "%s"
      },
      "assets": [
      ],
      "presentation_viewport": "playback",
      "width": %s,
      "height": %s,
      "x_coord": 0,
      "y_coord": 0
    }
  ]
}
"""

CAPTURE_SCENE = GenericMessage(
    type='json',
    message=SCENE_TEMPLATE % (
        CAPTURE_VIEWPORT,
        CAPTURE_WIDTH,
        CAPTURE_HEIGHT
    ))

HALF_SCALE_SCENE = GenericMessage(
    type='json',
    message=SCENE_TEMPLATE % (
        CAPTURE_VIEWPORT,
        CAPTURE_WIDTH / 2,
        CAPTURE_HEIGHT / 2
    ))


class ImageCapture:
    def __init__(self):
        self.images = []

    def handle_image(self, msg):
        self.images.append(msg)


class TestCaptureViewport(unittest.TestCase):
    def setUp(self):
        display = CAPTURE_DISPLAY
        geometry = ManagedWindow.lookup_viewport_geometry(CAPTURE_VIEWPORT)
        geometry_str = '{}x{}x24'.format(CAPTURE_WIDTH, CAPTURE_HEIGHT)

        self.capture = ImageCapture()
        self.capture_sub = rospy.Subscriber(CAPTURE_TOPIC, Image, self.capture.handle_image)

        # Run an Xvfb with the configured DISPLAY and geometry matching the
        # viewport exactly.
        self.xvfb = ProcController(
            ['Xvfb', display, '-noreset', '-screen', '0', geometry_str]
        )
        self.xvfb.start()
        rospy.sleep(3.0)

        self.pub = rospy.Publisher('/director/scene',
                                   GenericMessage,
                                   queue_size=100)
        rospy.sleep(1.0)

    def tearDown(self):
        self.pub.publish(BLANK_SCENE)
        rospy.sleep(0.5)
        self.xvfb.stop()

    def test_capture_image(self):
        self.assertEqual(0, len(self.capture.images))

        self.pub.publish(CAPTURE_SCENE)

        rospy.sleep(1.0)

        first = self.capture.images[0]
        self.assertEqual(CAPTURE_WIDTH, first.width)
        self.assertEqual(CAPTURE_HEIGHT, first.height)

        # Now try a different viewport size.
        self.pub.publish(HALF_SCALE_SCENE)

        rospy.sleep(1.0)

        last = self.capture.images[-1]
        self.assertEqual(CAPTURE_WIDTH / 2, last.width)
        self.assertEqual(CAPTURE_HEIGHT / 2, last.height)

        # We shouldn't get any more images after publishing blank scene.
        self.pub.publish(BLANK_SCENE)

        rospy.sleep(1.0)
        num_images = len(self.capture.images)
        rospy.sleep(1.0)
        self.assertEqual(num_images, len(self.capture.images))


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestCaptureViewport)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
