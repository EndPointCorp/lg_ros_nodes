#!/usr/bin/env python

PKG = 'lg_mirror'
NAME = 'test_aspect_scale_source'

from types import *
import unittest
from lg_mirror.utils import aspect_scale_source
from lg_common.msg import WindowGeometry


class TestAspectScale(unittest.TestCase):
    def check_dimensions(self, source, dest, expected_width, expected_height):
        width, height = aspect_scale_source(source, dest)
        self.assertEqual(expected_width, width)
        self.assertEqual(expected_height, height)
        self.assertTrue(type(width) is IntType)
        self.assertTrue(type(height) is IntType)

    def test_equality(self):
        sg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        dg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        self.check_dimensions(sg, dg, 1920, 1080)

    def test_portrait_to_landscape(self):
        sg = WindowGeometry(width=1080, height=1920, x=0, y=0)
        dg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        expected_width = int(round(1080 / (1920. / 1080.)))
        self.check_dimensions(sg, dg, expected_width, 1080)

    def test_landscape_to_portrait(self):
        sg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        dg = WindowGeometry(width=1080, height=1920, x=0, y=0)
        expected_height = int(round(1080 / (1920. / 1080.)))
        self.check_dimensions(sg, dg, 1080, expected_height)

    def test_scale_down(self):
        sg = WindowGeometry(width=3840, height=2160, x=0, y=0)
        dg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        self.check_dimensions(sg, dg, 1920, 1080)

    def test_scale_down_portrait_to_landscape(self):
        sg = WindowGeometry(width=2160, height=3840, x=0, y=0)
        dg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        expected_width = int(round(1080 / (3840. / 2160.)))
        self.check_dimensions(sg, dg, expected_width, 1080)

    def test_scale_down_landscape_to_portrait(self):
        sg = WindowGeometry(width=3840, height=2160, x=0, y=0)
        dg = WindowGeometry(width=1080, height=1920, x=0, y=0)
        expected_height = int(round(1080 / (3840. / 2160.)))
        self.check_dimensions(sg, dg, 1080, expected_height)

    def test_scale_up(self):
        sg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        dg = WindowGeometry(width=3840, height=2160, x=0, y=0)
        # Should never scale larger than source geometry.
        self.check_dimensions(sg, dg, 1920, 1080)

    def test_scale_up_portrait_to_landscape(self):
        sg = WindowGeometry(width=1080, height=1920, x=0, y=0)
        dg = WindowGeometry(width=3840, height=2160, x=0, y=0)
        # Should never scale larger than source geometry.
        self.check_dimensions(sg, dg, 1080, 1920)

    def test_scale_up_landscape_to_portrait(self):
        sg = WindowGeometry(width=1920, height=1080, x=0, y=0)
        dg = WindowGeometry(width=2160, height=3840, x=0, y=0)
        # Should never scale larger than source geometry.
        self.check_dimensions(sg, dg, 1920, 1080)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAspectScale)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
