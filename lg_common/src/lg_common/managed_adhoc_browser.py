#!/usr/bin/env python

import rospy
import commands

from lg_common import ManagedBrowser
from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState
from lg_common.msg import ApplicationState
from lg_common.msg import AdhocBrowser, AdhocBrowsers


class ManagedAdhocBrowser(ManagedBrowser):

    def __init__(self, geometry=None, log_level=0, slug=None, url=None, uid=None):
        self.slug = slug
        self.id = uid
        self.url = url
        self.geometry = geometry
        self.log_level = log_level

        super(ManagedAdhocBrowser, self).__init__(
            geometry=geometry, slug=slug, url=url, kiosk=True)

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "<id: %s, slug: %s, URL: %s, x: %s, y: %s, offset_x: %s, offset_y: %s>" % \
            (self.id, self.slug, self.url, self.geometry.width, self.geometry.height,
             self.geometry.x, self.geometry.y)

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

    def close(self):
        self.set_state(ApplicationState.STOPPED)
        self.clear_tmp_dir()
