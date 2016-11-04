#!/usr/bin/env python

import rospy
import lg_common

from lg_common.msg import GetScreenshot
from lg_common.msg import Screenshot

DEFAULT_BINARY = 'phantomjs'
DEFAULT_SCRIPT = 'screenshots.js'

class WebScreenshot:
    def __init__ (self, publisher, binary=DEFAULT_BINARY, script=DEFAULT_SCRIPT,
                  delay=250, user_agent=None):
        self.publisher = publisher
        self.binary = binary
        self.delay = delay
        self.script = script

        self.user_agent = user_agent
        self.call_tmpl = self.binary + ' ' + self.script;
        self.call_tmpl += ' --out base64';
        if self.user_agent:
            self.call_tmpl += ' --ua {}'.format(self.user_agent)
        if self.delay:
            self.call_tmpl += ' --delay {}'.format(self.delay)

    def take_screenshot:(self, search_screenshot):
        search = search_screenshot.search
        width = search_screenshot.width
        call = self.call_tmpl + ' --search {}'.format(search)
        if width:
            call += ' --width {}'.format(width)

        base64 = subprocess.check_output(call, shell=True)
        msg = Screenshot()
        msg.search = search
        msg.base64 = base64
        self.publisher.publish(msg)
