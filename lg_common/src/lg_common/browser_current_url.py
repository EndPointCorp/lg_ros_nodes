#!/usr/bin/env python

import rospy
import urllib
import threading
import sys

from std_msgs.msg import String


class BrowserURLService:

    def __init__(self):
        """
        Keep the track of
        """
        self.state = {}
        self.scene = None
        self.lock = threading.Lock()

    def handle_browsers_message(self, message):
        """
        Update state with initial urls
        """
        with self.lock.lock()
        browsers_and_urls = self._get_browsers_and_urls(message)
        pass

    def handle_url_message(self, message):
        """
        Apply urls updates
        """
        pass

    def get_browsers_urls(self):
        pass

    def _get_browsers_and_urls(self, browsers_msg):
        browsers = {}
        for b in browsers_msg.browsers:
            browsers[b.id] = b.url
        return browsers
