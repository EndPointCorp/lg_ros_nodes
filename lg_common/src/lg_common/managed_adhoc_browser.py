#!/usr/bin/env python
"""
TODO: Implement the changing geometry, and url.
"""
import rospy


from lg_common import ManagedBrowser
from lg_common.msg import ApplicationState
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
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
                app=True)

    def update_geometry(self, geometry):
        pass

    def update_url(self, url):
        import os
        self.url = url
        os.system('chromium-remote.py --slug={} --page-url="{}"'.format(self.slug, url))

    def close(self):
        self.set_state(ApplicationState.STOPPED)


class AdhocBrowserPool():
    """
    Handlers to all opened browsers
    Dict(id => ManagedAdhocBrowser)
    """
    browsers = {}

    def __init__(self):
        pass

    def handle_ros_message(self, data):
        """
        This function handles the ros messages, and manages the browsers.

        If the message has no AdhocBrowser messages in the array,
        close everything down.

        If the message has AdhocBrowser messages in the array:

        Check for an existing browser with the same id.
        If it exists, update the url and geometry.

        If no existing browser, create one.

        If an existing browser has an id that doesn't match the new message,
        shut it down and remove it from the tracked instances.

        Arguments:
            data: AdhocBrowsers, which is an array of AdhocBrowser

        """
        # Let's repack the data from the ros message, it will be easier to use it later in this for.
        # Dict(id => AdhocBrowser)
        new_browsers = {b.id: b for b in data.browsers}

        # And operate on sets of ids.
        current_browsers_ids = set(self.browsers.keys())
        new_browsers_ids = set(new_browsers.keys())

        # Create sets with ids to remove, create, and udpate.
        remove_ids = current_browsers_ids - new_browsers_ids
        create_ids = new_browsers_ids - current_browsers_ids
        update_ids = new_browsers_ids & current_browsers_ids

        # Close all browsers which are not on the new list, and remove them from the browsers dict.
        for id in remove_ids:
            self.browsers[id].close()
            del self.browsers[id]

        # Create new browsers, and add them to the browsers dict.
        for id in create_ids:
            d = new_browsers[id]
            url = d.url
            geometry = WindowGeometry(x=d.geometry.x,
                                      y=d.geometry.y,
                                      width=d.geometry.width,
                                      height=d.geometry.height)
            b = ManagedAdhocBrowser(geometry=geometry, slug=id, url=url)
            b.set_state(ApplicationState.VISIBLE)
            self.browsers[id] = b

        # Update existing browsers.
        for id in update_ids:
            b = self.browsers[id]
            d = new_browsers[id]

            geometry = WindowGeometry(x=d.geometry.x,
                                      y=d.geometry.y,
                                      width=d.geometry.width,
                                      height=d.geometry.height)
            b.update_geometry(geometry)
            b.update_url(d.url)
