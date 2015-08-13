#!/usr/bin/env python
import rospy

from lg_common import ManagedBrowser
from lg_common.msg import WindowGeometry
from lg_common import ManagedWindow
from lg_common.msg import ApplicationState
from lg_common.msg import ApplicationState
from lg_common.msg import AdhocBrowser, AdhocBrowsers


class ManagedAdhocBrowser(ManagedBrowser):

    def __init__(self, geometry, slug, url):
        self.slug = slug
        self.url = url
        self.geometry = geometry

        super(ManagedAdhocBrowser, self).__init__(
                geometry=geometry,
                slug=slug,
                url=url,
                kiosk=True)

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
        import os
        self.url = url
        os.system('chromium-remote.py --debug-host=localhost --debug-port={} --page-url="{}"'.format(self.debug_port, url))

    def close(self):
        self.set_state(ApplicationState.STOPPED)
