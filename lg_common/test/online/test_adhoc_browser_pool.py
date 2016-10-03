#!/usr/bin/env python

import time
import rospy
import unittest
from lg_common.msg import AdhocBrowser
from lg_common import AdhocBrowserPool


PKG = 'lg_common'
NAME = 'test_adhoc_browser_pool'

pool_created = False

class TestAdhocBrowserPool(unittest.TestCase):
    def setUp(self):
        if not pool_created:
            rospy.set_param('~rosbridge_secure', True)
            rospy.set_param('~rosbridge_port', 1234)
            self.pool = AdhocBrowserPool('center')
            pool_created = True

    def tearDown(self):
        pass

    def test_rosbridge_params_passed(self):
        test_browser_msg = AdhocBrowser()

        self.pool._create_browser('test_test', test_browser_msg)
        assert self.pool.browsers['test_test'] is not None

        url = self.pool.browsers['test_test'].url
        assert 'ros_instance_name=test_test' in url
        assert 'rosbridge_secure=1' in url
        assert 'rosbridge_port=1234' in url

    def test_allowed_url_params_passed(self):
        test_browser_msg = AdhocBrowser()
        test_browser_msg.allowed_urls.append('endpoint.com')
        test_browser_msg.allowed_urls.append('google.com')
        test_browser_msg.allowed_urls.append('test_url_pattern')

        self.pool._create_browser('test_test_2', test_browser_msg)
        assert self.pool.browsers['test_test_2'] is not None

        url = self.pool.browsers['test_test_2'].url
        assert 'allowed_urls=endpoint.com' in url
        assert 'allowed_urls=google.com' in url
        assert 'allowed_urls=test_url_pattern' in url

if __name__ == '__main__':
    rospy.init_node(NAME)
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowserPool)
