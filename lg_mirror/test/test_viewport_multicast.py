#!/usr/bin/env python

PKG = 'lg_mirror'
NAME = 'test_viewport_multicast'

import rospy
import unittest
from lg_mirror.utils import viewport_to_multicast_group
from lg_mirror.utils import get_mirror_port
from lg_mirror.constants import MULTICAST_GROUP, MULTICAST_PORT


class TestViewportMulticast(unittest.TestCase):
    def test_viewport_to_multicast_group(self):
        addr = viewport_to_multicast_group('first')
        expected = MULTICAST_GROUP.format(1)
        self.assertEquals(expected, addr)

        addr = viewport_to_multicast_group('second')
        expected = MULTICAST_GROUP.format(2)
        self.assertEquals(expected, addr)

        addr = viewport_to_multicast_group('third')
        expected = MULTICAST_GROUP.format(3)
        self.assertEquals(expected, addr)

        # Must raise on missing key.
        with self.assertRaises(KeyError):
            addr = viewport_to_multicast_group('whoops')

        # Must raise on no viewports.
        rospy.delete_param('/viewport')
        with self.assertRaises(KeyError):
            addr = viewport_to_multicast_group('first')

    def test_multicast_port(self):
        port = get_mirror_port()


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestViewportMulticast)


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
