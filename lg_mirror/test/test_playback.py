#!/usr/bin/env python

PKG = 'lg_mirror'
NAME = 'test_playback'

import gc
import unittest
import weakref

from lg_common.msg import WindowGeometry
from lg_mirror import MirrorException
from lg_mirror.playback import MirrorPlayback


class TestMirrorPlayback(unittest.TestCase):
    def test_playback_lifecycle(self):
        """
        Test the lifecycle without inspecting that a process is running.
        """
        geometry = WindowGeometry(width=1920, height=1080, x=0, y=0)
        pb = MirrorPlayback(
            instance_name=NAME,
            janus_url='http://localhost:8188/janus',
            source_viewport='test',
            geometry=geometry
        )

        pb.start()
        with self.assertRaises(MirrorException):
            pb.start()

        pb_ref = weakref.ref(pb)

        pb.stop()
        with self.assertRaises(MirrorException):
            pb.stop()

        # XXX: failure to gc is likely a ManagedApplication bug
        # See #262
        #pb = None
        #gc.collect()
        #self.assertIsNone(pb_ref())


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestMirrorPlayback)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
