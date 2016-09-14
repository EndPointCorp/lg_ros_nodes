#!/usr/bin/env python

import time
import rospy
import unittest
import lg_common
from lg_common.msg import AdhocBrowser


PKG = 'lg_common'
NAME = 'test_adhoc_browser_pool'


class AdhocBrowserPoolStub():
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.state = None

    def set_state(self, state):
        self.state = state


lg_common.ManagedAdhocBrowser = AdhocBrowserPoolStub


class TestAdhocBrowserPool(unittest.TestCase):
    def setUp(self):
        self.pool = AdhocBrowserPool('center')

    def tearDown(self):
        pass

    def test_rosbridge_params_passed:
        test_browser_msg = AdhocBrowser()

        self.pool._create_browser(self, 'test_test', test_browser_msg, initial_state=None)
        assert self.pool.browsers['test_test'] is not None


if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowserPool)
