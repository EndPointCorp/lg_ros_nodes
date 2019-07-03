#!/usr/bin/env python
PKG = 'lg_common'
NAME = 'test_combine_viewport_geometries'

import rospy
import unittest
from lg_panovideo import util
from lg_common import ManagedWindow


class TestCombineViewportGeometries(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def _assert_equal_geometries(self, a, b):
        self.assertEqual(a.x, b.x)
        self.assertEqual(a.y, b.y)
        self.assertEqual(a.width, b.width)
        self.assertEqual(a.height, b.height)

    def test_combine_all_geometries(self):
        geometry_names = ['touchscreen', 'left_one', 'center', 'right_one']
        combined_geometry = util.combine_viewport_geometries(geometry_names)

        expected_geometry = ManagedWindow.lookup_viewport_geometry('expected_all')

        self._assert_equal_geometries(expected_geometry, combined_geometry)

    def test_combine_portrait_geometries(self):
        geometry_names = ['left_one', 'center', 'right_one']
        combined_geometry = util.combine_viewport_geometries(geometry_names)

        expected_geometry = ManagedWindow.lookup_viewport_geometry('expected_portraits')

        self._assert_equal_geometries(expected_geometry, combined_geometry)


if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCombineViewportGeometries)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
