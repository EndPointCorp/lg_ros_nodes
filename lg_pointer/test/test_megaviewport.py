#!/usr/bin/env python

PKG = 'lg_pointer'
NAME = 'test_megaviewport'

import math
import unittest

import rospy
from lg_pointer import MegaViewport

TEST_VIEWPORTS = ['left', 'center', 'right']
TEST_ARC_WIDTH = math.pi / 2


class MockPublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class TestMegaViewport(unittest.TestCase):
    def setUp(self):
        self.mvp = MegaViewport(TEST_VIEWPORTS, TEST_ARC_WIDTH)

    def test_center(self):
        vp, vpx, vpy = self.mvp.orientation_to_coords(0, 0)

        self.assertEqual(vp, TEST_VIEWPORTS[1])
        self.assertAlmostEqual(vpx, 0.5)
        self.assertAlmostEqual(vpy, 0.5)

    def test_offscreen(self):
        vp, vpx, vpy = self.mvp.orientation_to_coords(TEST_ARC_WIDTH, 0)

        self.assertEqual(vp, '')
        self.assertEqual(vpx, 0)
        self.assertEqual(vpy, 0)


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestMegaViewport)
