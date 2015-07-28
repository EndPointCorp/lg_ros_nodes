import rospy

from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
from lg_common.msg import AdhocBrowser, AdhocBrowsers


class AdhocBrowserPool():
    """
    Handlers to all opened browsers
    Dict(id => ManagedAdhocBrowser)
    """
    browsers = {}

    def __init__(self):
        self.browsers = {}

    def _unpack_incoming_browsers(self, browsers):
        """
        Return dict(id: AdhocBrowser) with all AdhocBrowsers
        """
        return {b.id: b for b in browsers}

    def _get_current_browsers_ids(self):
        return set(self.browsers.keys())

    def _get_browsers_ids_to_remove(self, incoming_browsers_ids):
        """
        returns a list of browsers that are not on the list of incoming message
        this means that it returns list of browsers to remove
        """
        browsers_ids_to_remove = self._get_current_browsers_ids() - incoming_browsers_ids
        return list(browsers_ids_to_remove)

    def _get_browsers_ids_to_create(self, incoming_browsers_ids):
        """
        returns a list of browsers that are not on the list of current browsers
        this means that it returns list of browsers to create
        """
        browsers_ids_to_create = incoming_browsers_ids - self._get_current_browsers_ids()
        return list(browsers_ids_to_create)

    def _get_browsers_to_update(self, incoming_browsers_ids):
        """
        returns a list of browsers that are not on the list of incoming message
        this means that it returns list of browsers to remove
        """
        browser_ids_to_update = self._get_current_browsers_ids() & incoming_browsers_ids
        return list(browser_ids_to_update)

    def _remove_browser(self, browser_id):
        """
        call .close() on browser object and delete the object
        """
        self.browsers[browser_id].close()
        del self.browsers[browser_id]

    def _create_browser(self, new_browser_id, new_browser):
        """
        Create new browser instance
        """

        geometry = WindowGeometry(x=new_browser.geometry.x,
                                  y=new_browser.geometry.y,
                                  width=new_browser.geometry.width,
                                  height=new_browser.geometry.height)
        browser_to_create = ManagedAdhocBrowser(geometry=geometry,
                                                slug=new_browser_id,
                                                url=new_browser.url)
        browser_to_create.set_state(ApplicationState.VISIBLE)

        self.browsers[browser_id] = browser_to_create
        return True

    def _update_browser(self, browser_id, updated_browser):
        """
        Update existing browser instance
        """
        old_browser = self.browsers[browser_id]

        new_geometry = WindowGeometry(x=updated_browser.geometry.x,
                                      y=updated_browser.geometry.y,
                                      width=updated_browser.geometry.width,
                                      height=updated_browser.geometry.height)

        if new_geometry != old_geometry:
            """
            TODO(wz): make url updatable
            """
            rospy.logdebug("Updating geometry of browser id %s" % browser_id)
            old_browser.update_geometry(new_geometry)

        if new_url != old_url:
            rospy.logdebug("Updating URL of browser id %s from %s to %s" % (browser_id, old_url, new_url))
            old_browser.update_url(updated_browser.url)

        return True

    def handle_ros_message(self, data):
        """
        - if message has no AdhocBrowser messages in the array,
        close everything down.
        - if the message has AdhocBrowser messages in the array:
        -- check for an existing browser with the same id.
        --- if it exists, update the url and geometry.
        --- if no existing browser, create one.
        --- if an existing browser has an id that doesn't match the new message, then
            shut it down and remove it from the tracked instances.

        Arguments:
            data: AdhocBrowsers, which is an array of AdhocBrowser

        """

        incoming_browsers      = self._unpack_incoming_browsers(data.browsers)
        incoming_browsers_ids  = set(incoming_browsers.keys())

        #remove
        for browser_id in self._get_browsers_ids_to_remove(incoming_browsers_ids):
            self._remove_browser(browser_id)

        #create
        for browser_id in self._get_browsers_ids_to_create(incoming_browsers_ids):
            self._create_browser(browser_id, incoming_browsers[browser_id])

        #update
        for browser_id in self._get_browsers_ids_to_update(incoming_browsers_ids):
            self._update_browser(browser_id, incoming_browsers[browser_id])

        return True

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
