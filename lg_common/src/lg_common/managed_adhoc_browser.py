#!/usr/bin/env python

import rospy
import commands
import time
import json

from lg_common import ManagedBrowser
from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState
from lg_common.msg import ApplicationState
from lg_common.msg import AdhocBrowser, AdhocBrowsers


class ManagedAdhocBrowser(ManagedBrowser):
    def __init__(self, geometry=None, log_level=0, command_line_args=[],
                 default_args_removal=[],
                 extensions=[], binary='/usr/bin/google-chrome',
                 user_agent=None, slug=None, url=None, uid=None,
                 scene_slug=None, preload=False, kiosk=True):

        self.scene_slug = scene_slug
        self.slug = slug
        self.id = uid
        self.url = url
        self.geometry = geometry
        self.preload = preload
        self.log_level = log_level
        self.user_agent = user_agent
        self.binary = binary
        self.command_line_args = command_line_args
        self.default_args_removal = default_args_removal
        self.extensions = extensions
        self.timestamp = int(time.time())

        # we're pass up only the stuff that matters during browser launch
        super(ManagedAdhocBrowser, self).__init__(
            slug=slug,
            url=url,
            geometry=geometry,
            user_agent=user_agent,
            command_line_args=command_line_args,
            default_args_removal=default_args_removal,
            extensions=extensions,
            binary=binary,
            log_level=log_level,
            kiosk=kiosk)

    def __str__(self):
        return json.dumps({
            "slug": self.slug,
            "url": self.url,
            "uid": self.id,
            "x_offset": self.geometry.x,
            "y_offset": self.geometry.y,
            "width": self.geometry.width,
            "height": self.geometry.height,
            "extensions": self.extensions,
            "binary": self.binary,
            "user_agent": self.user_agent,
            "command_line_args": self.command_line_args,
            "default_args_removal": self.default_args_removal,
            "timestamp": self.timestamp
        })

    def __repr__(self):
        return self.__str__()

    def update_geometry(self, geometry):
        """
        Updates the geometry and converges window with new settings
        """
        self.window.geometry = geometry
        self.window.converge()

    def update_url(self, url):
        """
        Updates URL of the browser using 'chromium-remote.py' script
        """
        try:
            cmd = 'chromium-remote.py --page-retries=10 --debug-host=localhost --debug-port={} --page-url="{}"'.format(self.debug_port, url)
            status, output = commands.getstatusoutput(cmd)
            if status == 0:
                self.url = url
                rospy.loginfo("Successfully executed URL change command (%s) for browser: %s, old_url: %s, new_url: %s" % (cmd, self.slug, self.url, url))
                return True
            else:
                rospy.logerr("URL change command: %s, returned a status code: %s and output %s" % (cmd, status, output))
                return False
        except Exception, e:
            rospy.logerr("URL change command: %s, returned a status code: %s and output %s because %s" % (cmd, status, output, e))
            return False

    def close(self, delay=None):
        self.set_state(ApplicationState.STOPPED)
        self.clear_tmp_dir()
