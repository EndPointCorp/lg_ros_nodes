#!/usr/bin/env python3

PKG = 'lg_common'
NAME = 'test_adhoc_browser_director_bridge'

import rospy
import unittest

from lg_msg_defs.msg import GetScreenshot
from lg_msg_defs.msg import Screenshot
from lg_screenshot import WebScreenshot


class MockScreenshotPublisher:
    def __init__(self):
        self.published_messages = []

    def publish(self, screenshot):
        self.published_messages.append(screenshot)


class TestWebScreenshot(unittest.TestCase):
    def setUp(self):
        self.publisher = MockScreenshotPublisher()
        self.node = WebScreenshot(self.publisher,
                                  binary='echo',
                                  script='script',
                                  delay=100)

    def test_take_screenshot(self):
        msg = GetScreenshot()
        msg.url = 'http://myhost.me/screenshot?blah=blah'
        msg.page_width = 1200
        self.node.take_screenshot(msg)

        self.assertEqual(1, len(self.publisher.published_messages))
        #self.assertEqual(self.publisher.published_messages, 'asdasd')
        self.assertTrue('--url {}'.format(msg.url).encode('utf-8') in self.publisher.published_messages[0].base64)
        self.assertTrue('--out base64'.format(msg.url).encode('utf-8') in self.publisher.published_messages[0].base64)
        self.assertTrue('--delay 100'.encode('utf-8') in self.publisher.published_messages[0].base64)
        self.assertTrue('--width 1200'.encode('utf-8') in self.publisher.published_messages[0].base64)
        self.assertTrue('--silent true'.encode('utf-8') in self.publisher.published_messages[0].base64)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestWebScreenshot)
