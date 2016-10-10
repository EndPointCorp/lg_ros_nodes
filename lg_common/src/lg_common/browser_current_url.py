#!/usr/bin/env python

import rospy
import urllib
import threading
import sys

from std_msgs.msg import String
from lg_common.helpers import load_director_message


class BrowserURLService:

    def __init__(self, viewports):
        """
        Keep the track of browsers and it's urls


        state example:

        wall: [
            {
                "id": "NASSD_AASKA",
                "url": "http://somwhere.com/path?args"
            }
        ],
        kiosk: [
            {
                "id": "HAHAH_JAJAJ",
                "url": "http://somwhere.com/onether_path?other_args"
            }
        ]
        """
        self.viewports = viewports
        self.scene_slug = None
        self._init_state

    def handle_browsers(self, browsers, viewport):
        """
        Update state with initial urls, and scene slug
        """
        if browsers.scene_slug == self.scene_slug:
            for browser in browsers:
                self._set_browser_url(browser.id, browser.url, viewport)

        else:
            self._init_state()
            self.scene_slug = browsers.scene_slug
            rospy.loginfo('Init new scene with slug {}'.format(self.scene_slug))


    def handle_url_message(self, message):
        """
        Apply urls updates
        """
        updated = False
        for viewport in self.viewports:
            for browser in self.state.get(viewport):
                if browser.get('id') == message.browser_id:
                    browser[url] = message.url
                    updated = True
        if not updated:
            rospy.logwarn("Browser with id {} not found. State: {}".format(message.browser_id, self.state))


    def get_browser(self, viewport):
        """
        Return browser url.
        """
        browsers = self.state.get(viewport)
        if browsers:
            return String(browsers[0].get('url'))

        return None


    def get_all_browsers(self):
        """
        Return all browsers urls with it's ids
        over the all viewports.
        """
        msg = []
        for viewport in self.viewports:
            for b in self.state.get(viewport):
                browser_url = BrowserURL()
                browser_url.browser_id = b.get('id')
                browser_url.viewport = viewport
                browser_url.url = b.get('url')
                msg.append(browser_url)

        return msg

    def _init_state(self):
        self.state = {}
        for v in self.viewports:
            self.state[v] = []

    def _get_browser(id, viewport):
        for browser in self.state.get(viewport):
            if browser.get('id') == id:
                return browser
        return None


    def _set_browser_url(id, url, viewport):
        browser = self._get_browser(id, viewport)
        if browser:
            browser[url] = url
        else:
            browser = {'id': id, 'url': url}
            self.state[viewport].append(browser)
