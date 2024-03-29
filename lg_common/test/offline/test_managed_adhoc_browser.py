#!/usr/bin/env python3

PKG = 'lg_common'
NAME = 'test_adhoc_browser_director_bridge'

import rospy
import unittest

from lg_common import ManagedAdhocBrowser
from lg_msg_defs.msg import WindowGeometry, ApplicationState

from lg_common.logger import get_logger
logger = get_logger('test_managed_adhoc_browser')


class TestManagedAdhocBrowser(unittest.TestCase):
    def setUp(self):
        """
        - instantiate ManagedAdhocBrowser
        - run asserts on object's parameters
        """

        self.width = 1000
        self.height = 1001
        self.x = 1002
        self.y = 1003
        self.slug = 'testing_slug'
        self.url = 'http://justtesting.com'

        self.geometry = WindowGeometry(width=self.width,
                                       height=self.height,
                                       x=self.x,
                                       y=self.y)
        self.mab = ManagedAdhocBrowser(geometry=self.geometry,
                                       slug=self.slug,
                                       url=self.url)

        super(ManagedAdhocBrowser, self.mab).__init__(
            geometry=self.geometry,
            slug=self.slug,
            url=self.url,
            kiosk=True)
        logger.debug("This is mab: %s" % self.mab.__dict__)

    def test_1_run_basic_asserts(self):
        """
        Check if managed adhoc browsers parameters are set properly
        """
        self.assertEqual(self.width, self.mab.geometry.width)
        self.assertEqual(self.height, self.mab.geometry.height)
        self.assertEqual(self.x, self.mab.geometry.x)
        self.assertEqual(self.y, self.mab.geometry.y)
        self.assertEqual(self.slug, self.mab.slug)
        self.assertEqual(self.url, self.mab.url)
        self.assertEqual(ApplicationState.STOPPED, self.mab.state)
        self.assertTrue((self.url in self.mab.cmd))
        self.assertTrue(('--kiosk' in self.mab.cmd))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestManagedAdhocBrowser)
