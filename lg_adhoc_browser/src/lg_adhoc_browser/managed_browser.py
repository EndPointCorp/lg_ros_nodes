#!/usr/bin/env python
"""
TODO: Implement the changing geometry, and url.
"""

from lg_common import ManagedBrowser
from lg_common.msg import ApplicationState


class ManagedAdhocBrowser(ManagedBrowser):

    def __init__(self, geometry, slug, url):
        self.slug = slug
        self.url = url
        self.geometry = geometry

        super(ManagedAdhocBrowser, self).__init__(
                geometry=geometry,
                slug=slug,
                url=url,
                app=True)

    def update_geometry(self, geometry):
        pass

    def update_url(self, url):
        import os
        self.url = url
        os.system('chromium-remote.py --slug={} --page-url="{}"'.format(self.slug, url))

    def close(self):
        self.set_state(ApplicationState.STOPPED)
