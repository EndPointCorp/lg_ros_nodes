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
from lg_mirror.utils import viewport_to_multicast_group
from lg_mirror.utils import get_mirror_port
from appctl_support import ProcController
from interactivespaces_msgs.msg import GenericMessage

CAPTURE_HOST = os.environ.get('CAPTURE_HOST')
CAPTURE_VIEWPORT = os.environ.get('CAPTURE_VIEWPORT')
CAPTURE_DISPLAY = os.environ.get('DISPLAY')
CAPTURE_WIDTH = int(os.environ.get('CAPTURE_WIDTH'))
CAPTURE_HEIGHT = int(os.environ.get('CAPTURE_HEIGHT'))

SOCK_TIMEOUT = 3  # seconds

BLANK_SCENE = GenericMessage(
    type='json',
    message='{}'
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
      },
      "assets": [
        "viewport://%s"
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

# Struct for unpacking RTP headers.
RTP_HEADER = struct.Struct('!HHII')
# Version 2 is RFC3550.
RTP_EXPECTED_VERSION = 2
# Payload 96 is the first dynamic type.
RTP_EXPECTED_PAYLOAD = 96

GST_PROC_NAME = 'gst-launch-1.0'


def create_listen_sock(addr, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setblocking(0)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((addr, port))
    # Join multicast group if applicable.
    mreq = struct.pack('4sl', socket.inet_aton(addr), socket.INADDR_ANY)
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    except socket.error:
        pass
    return sock


def find_gstreamer_procs():
    """
    Find gstreamer procs whose grandparent pid is equal to our parent pid.

    This provides isolation from ambient gstreamers.
    """
    myppid = os.getppid()

    def match_proc(proc):
        if proc.name != GST_PROC_NAME:
            return False

        if proc.ppid == 0:
            return False

        pproc = psutil.Process(proc.ppid)
        if pproc.ppid == 0:
            return False

        gpid = pproc.ppid
        if gpid != myppid:
            return False

        return True

    return filter(match_proc, psutil.process_iter())


class TestCaptureViewport(unittest.TestCase):
    def setUp(self):
        display = CAPTURE_DISPLAY
        geometry = ManagedWindow.lookup_viewport_geometry(CAPTURE_VIEWPORT)
        geometry_str = '{}x{}x24'.format(CAPTURE_WIDTH, CAPTURE_HEIGHT)

        # Multicast is disabled in this test due to firewall complications.
        #self.addr = viewport_to_multicast_group(CAPTURE_VIEWPORT)

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

    def test_capture_socket(self):
        addr = CAPTURE_HOST
        port = get_mirror_port()
        sock = create_listen_sock(addr, port)

        self.pub.publish(CAPTURE_SCENE)

        ready = select.select([sock], [], [], SOCK_TIMEOUT)
        self.assertTrue(ready[0], 'Timeout waiting for data on socket')

        buf = sock.recv(RTP_HEADER.size)

        # Verify some expectations about the RTP header.
        rtp_header = RTP_HEADER.unpack_from(buf)
        rtp_version = (rtp_header[0] & 0xC000) >> 14
        self.assertEqual(RTP_EXPECTED_VERSION, rtp_version)
        rtp_payload = (rtp_header[0] & 0x007F)
        self.assertEqual(RTP_EXPECTED_PAYLOAD, rtp_payload)

        sock.close()

        self.pub.publish(BLANK_SCENE)
        rospy.sleep(0.5)

        sock = create_listen_sock(addr, port)

        ready = select.select([sock], [], [], SOCK_TIMEOUT)
        self.assertFalse(ready[0], 'There should not be data on the socket')

    def test_capture_proc(self):
        self.pub.publish(CAPTURE_SCENE)
        rospy.sleep(1.0)

        gst_procs = find_gstreamer_procs()
        self.assertEqual(1, len(gst_procs))
        first_pid = gst_procs[0].pid

        self.pub.publish(CAPTURE_SCENE)
        rospy.sleep(1.0)

        gst_procs = find_gstreamer_procs()
        self.assertEqual(1, len(gst_procs))
        second_pid = gst_procs[0].pid

        self.assertEqual(first_pid, second_pid,
                         'Should not restart on duplicate scene')

        self.pub.publish(HALF_SCALE_SCENE)
        rospy.sleep(1.0)

        gst_procs = find_gstreamer_procs()
        self.assertEqual(1, len(gst_procs))
        third_pid = gst_procs[0].pid

        self.assertNotEqual(second_pid, third_pid,
                            'Should restart on different target geometry')

        self.pub.publish(BLANK_SCENE)
        rospy.sleep(1.0)

        gst_procs = find_gstreamer_procs()
        self.assertEqual(0, len(gst_procs),
                         'Should stop on blank scene')


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestCaptureViewport)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
