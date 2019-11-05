#!/usr/bin/env python3

PKG = 'lg_common'
NAME = 'test_adhoc_browser_uscs_initial_state'

import rospy
import unittest
import json

from lg_common import InteractiveSpacesMessagesFactory
from lg_common.srv import BrowserPool
from interactivespaces_msgs.msg import GenericMessage


class TestAdhocBrowser(unittest.TestCase):
    def setUp(self):
        rospy.wait_for_service('/browser_service/touchscreen', 5)
        self.touchscreen_browser_service = rospy.ServiceProxy('/browser_service/touchscreen', BrowserPool)
        self.message_factory = InteractiveSpacesMessagesFactory()
        # director scene publisher
        self.director_publisher = rospy.Publisher(
            '/director/scene',
            GenericMessage,
            queue_size=3
        )
        rospy.init_node("uscs_initial_state_testing", anonymous=True)
        rospy.sleep(3)

    def test_1_adhoc_browser_pool_pulled_initial_state_from_uscs_service(self):
        rospy.sleep(3)
        rospy.sleep(3)
        browsers_on_touchscreen = self.touchscreen_browser_service('{}').state
        browsers_on_touchscreen = json.loads(browsers_on_touchscreen)
        self.assertEqual(len(browsers_on_touchscreen), 1)

        for browser_id, browser_data in list(browsers_on_touchscreen.items()):
            self.assertEqual(browser_data['binary'], '/usr/bin/google-chrome-stable')

        #cleanup
        self.director_publisher.publish(self.message_factory._get_message('test_no_browsers_msg'))
        rospy.sleep(3)

        # make sure browsers went away
        browsers_on_touchscreen = self.touchscreen_browser_service('{}').state
        browsers_on_touchscreen = json.loads(browsers_on_touchscreen)
        self.assertEqual(len(browsers_on_touchscreen), 0)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAdhocBrowser)
