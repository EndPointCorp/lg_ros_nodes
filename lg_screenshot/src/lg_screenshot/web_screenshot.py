#!/usr/bin/env python

import rospy

from lg_screenshot.msg import GetScreenshot
from lg_screenshot.msg import Screenshot

DEFAULT_BINARY = 'phantomjs'
DEFAULT_SCRIPT = 'screenshots.js'


class WebScreenshot:
    def __init__(self, publisher, binary=DEFAULT_BINARY, script=DEFAULT_SCRIPT,
                 delay=250, user_agent=None):
        self.publisher = publisher
        self.binary = binary
        self.delay = delay
        self.script = rospkg.RosPack().get_path('lg_sceenshot') + "/webapps/" + script

        self.user_agent = user_agent
        self.call_tmpl = [self.binary]
        self.call_tmpl.extend(self.script)
        self.call_tmpl.extend('--out base64')
        if self.user_agent:
            self.call_tmpl.extend('--ua {}'.format(self.user_agent))
        if self.delay:
            self.call_tmpl.extend('--delay {}'.format(self.delay))

        rospy.loginfo("Initialized WebScreenshot with %s args" % self.call_tmpl)

    def take_screenshot(self, search_screenshot):
        url = search_screenshot.url
        width = search_screenshot.width
        call = [].extend(self.call_tmpl)
        call.extend('--url {}'.format(url))
        if width:
            call.extend('--width {}'.format(width))

        base64 = subprocess.check_output(' '.join(call), shell=True)
        msg = Screenshot()
        msg.search = search
        msg.base64 = base64
        self.publisher.publish(msg)
