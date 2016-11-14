#!/usr/bin/env python

import rospy

from lg_screenshot.msg import GetScreenshot
from lg_screenshot.msg import Screenshot
import subprocess
import rospkg

DEFAULT_BINARY = 'phantomjs'
DEFAULT_SCRIPT = 'screenshots.js'


class WebScreenshot:
    def __init__(self, publisher, binary=DEFAULT_BINARY, script=DEFAULT_SCRIPT,
                 delay=250, user_agent=None):
        self.publisher = publisher
        self.binary = binary
        self.delay = delay
        self.script = rospkg.RosPack().get_path('lg_screenshot') + "/webapps/" + script

        self.user_agent = user_agent
        self.call_tmpl = [self.binary]
        self.call_tmpl.extend([self.script])
        self.call_tmpl.extend(['--out base64'])
        if self.delay:
            self.call_tmpl.extend(['--delay {}'.format(self.delay)])

        rospy.loginfo("Initialized WebScreenshot with %s args" % self.call_tmpl)

    def take_screenshot(self, search_screenshot):
        url = search_screenshot.url
        width = search_screenshot.page_width
        if search_screenshot.user_agent:
            user_agent = search_screenshot.user_agent
        else:
            user_agent = self.user_agent

        call = []
        call.extend(self.call_tmpl)
        call.extend(['--ua \'{}\''.format(user_agent)])
        call.extend(['--url {}'.format(url)])
        call.extend(['--silent true'])
        if width:
            call.extend(['--width {}'.format(width)])
        if search_screenshot.zoom:
            call.extend(['--zoom {}'.format(search_screenshot.zoom)])
        if search_screenshot.scripts:
            call.extend(['--scripts {}'.format(' '.join(search_screenshot.scripts))])

        rospy.loginfo('Call pahntom js for a screenshot with args: {}'.format(call))
        base64 = subprocess.check_output(' '.join(call), shell=True)
        msg = Screenshot()
        msg.url = url
        msg.base64 = base64
        self.publisher.publish(msg)
        rospy.loginfo("Made screenshot for %s" % url)
