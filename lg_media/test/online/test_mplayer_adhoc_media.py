#!/usr/bin/env python3

PKG = 'lg_media'
NAME = 'test_mplayer_adhoc_media'

import rospy
import unittest

from lg_media import ManagedMplayer
from lg_msg_defs.msg import WindowGeometry, ApplicationState
from lg_common import ManagedWindow


class TestManagedAdhocBrowser(unittest.TestCase):
    def setUp(self):
        """
        - instantiate ManagedMplayer
        - run asserts on object's parameters
        """

        self.width = 1000
        self.height = 1001
        self.x = 1002
        self.y = 1003
        self.slug = 'testing_slug'
        self.url = 'http://lg-head/lg/assets/videos/bunny.mp4'
        self.fifo_path = '/tmp/asd_lol'
        self.cmd = '-idle -slave'

        self.geometry = WindowGeometry(width=self.width,
                                       height=self.height,
                                       x=self.x,
                                       y=self.y)

        self.window = ManagedWindow(geometry=self.geometry,
                                    w_instance="Mplayer \\({}\\)".format(self.slug),
                                    w_class="Mplayer \\({}\\)".format(self.slug)
                                    )

        self.mam = ManagedMplayer(window=self.window,
                                  fifo_path=self.fifo_path,
                                  slug=self.slug,
                                  url=self.url)

        super(ManagedMplayer, self.mam).__init__(
            cmd=self.cmd,
            window=self.window)
        rospy.logdebug("This is mam: %s" % self.mam.__dict__)

    def test_1_run_basic_asserts(self):
        """
        Check if mplayer parameters are set properly
        """
        self.assertEqual(self.width, self.mam.window.geometry.width)
        self.assertEqual(self.height, self.mam.window.geometry.height)
        self.assertEqual(self.x, self.mam.window.geometry.x)
        self.assertEqual(self.y, self.mam.window.geometry.y)
        self.assertEqual(self.slug, self.mam.slug)
        self.assertEqual(self.url, self.mam.url)
        self.assertEqual(self.fifo_path, self.mam.fifo_path)
        self.assertEqual(ApplicationState.STOPPED, self.mam.state)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestManagedAdhocBrowser)
