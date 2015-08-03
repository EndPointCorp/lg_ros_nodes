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
        self.geometry = self._add_offset_to_geometry(geometry)

        super(ManagedAdhocBrowser, self).__init__(
                geometry=geometry,
                slug=slug,
                url=url,
                kiosk=True)

    def _add_offset_to_geometry(self, supplied_geometry):
        """
        Adhoc browser needs geometry that's honoring viewport offsets
        This method will add offsets to original geometry
        """
        viewport_geometry = ManagedWindow.get_viewport_geometry()
        supplied_geometry.x += viewport_geometry.x
        supplied_geometry.y += viewport_geometry.y

        return supplied_geometry

    def update_geometry(self, geometry):
        """
        ManagedAdhocBrowser << ManagedBrowser << ManagedApplication.window.geometry = 'WxH[+-]X[+-]Y'
        ManagedAdhocBrowser << ManagedBrowser << ManagedApplication.window.converge()
        """
        self.window.geometry = self._add_offset_to_geometry(geometry)
        self.window.converge(geometry)

    def update_url(self, url):
        import os
        self.url = url
        os.system('chromium-remote.py --debug-host=localhost --debug-port={} --page-url="{}"'.format(self.debug_port, url))

    def close(self):
        self.set_state(ApplicationState.STOPPED)
