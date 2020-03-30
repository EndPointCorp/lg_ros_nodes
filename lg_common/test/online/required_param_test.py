#!/usr/bin/env python3

PKG = 'lg_common'
NAME = 'test_required_param'

import rospy
import unittest

from lg_common.helpers import required_param

TEST_KEY = "test_key"
TEST_VALUE = "test_value"
TEST_MISSING_KEY = "asdfhjkl"


class TestRequiredParam(unittest.TestCase):
    def test_get_value(self):
        value = required_param(TEST_KEY)
        self.assertEqual(TEST_VALUE, value)

    def test_missing_key(self):
        with self.assertRaises(KeyError):
            required_param(TEST_MISSING_KEY)

    def test_coerce(self):
        value = required_param(TEST_KEY, list)
        self.assertEqual(list, type(value))
        self.assertEqual(list(TEST_VALUE), value)

    def test_bad_coerce(self):
        with self.assertRaises(ValueError):
            required_param(TEST_KEY, float)


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestRequiredParam)
