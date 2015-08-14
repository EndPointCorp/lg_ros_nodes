import rospy

from lg_common import ManagedAdhocBrowser
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
from lg_common.msg import AdhocBrowser, AdhocBrowsers


class AdhocBrowserPool():
    """
    Handles browser pool in self.browsers
    Dict(id => ManagedAdhocBrowser)
    """
    def __init__(self, viewport_name):
        """
        AdhocBrowserPool manages a pool of browsers on one viewport.
        """
        self.viewport_name = viewport_name
        self.browsers = {}

    def _unpack_incoming_browsers(self, browsers):
        """
        Return dict(id: AdhocBrowser) with all AdhocBrowsers with ids as keys
        """
        return {b.id: b for b in browsers}

    def _get_current_browsers_ids(self):
        """
        Returns a mathematical set of ids of currently running browsers
        """
        return set(self.browsers.keys())

    def _get_browsers_ids_to_remove(self, incoming_browsers_ids):
        """
        Returns a list of browsers that are not on the list of incoming message.
        Effectively it's a list of ids of browsers that should be killed.
        """
        browsers_ids_to_remove = self._get_current_browsers_ids() - incoming_browsers_ids
        return list(browsers_ids_to_remove)

    def _get_browsers_ids_to_create(self, incoming_browsers_ids):
        """
        Returns a list of browsers that are not on the list of current browsers.
        This means that it returns list of browsers to create.
        """
        browsers_ids_to_create = incoming_browsers_ids - self._get_current_browsers_ids()
        return list(browsers_ids_to_create)

    def _get_browsers_ids_to_update(self, incoming_browsers_ids):
        """
        Returns a list of browsers that **are** on the list of incoming message.
        This means that it returns list of browsers that possibly needs URL or geometry update.
        """
        browser_pool_ids_to_update = self._get_current_browsers_ids() & incoming_browsers_ids
        return list(browser_pool_ids_to_update)

    def _remove_browser(self, browser_pool_id):
        """
        call .close() on browser object and cleanly delete the object
        """
        rospy.loginfo("POOL %s: Removing browser with id %s" % (self.viewport_name, browser_pool_id))
        self.browsers[browser_pool_id].close()
        del self.browsers[browser_pool_id]
        rospy.loginfo("POOL %s: state after removal: %s" % (self.viewport_name, self.browsers))

    def _create_browser(self, new_browser_pool_id, new_browser):
        """
        Create new browser instance with desired geometry.
        """
        geometry = WindowGeometry(x=new_browser.geometry.x,
                                  y=new_browser.geometry.y,
                                  width=new_browser.geometry.width,
                                  height=new_browser.geometry.height)

        browser_to_create = ManagedAdhocBrowser(geometry=geometry,
                                                slug=new_browser_pool_id,
                                                url=new_browser.url)
        browser_to_create.set_state(ApplicationState.VISIBLE)

        rospy.loginfo("POOL %s: Creating new browser %s with id %s" % (self.viewport_name, new_browser, new_browser_pool_id))
        self.browsers[new_browser_pool_id] = browser_to_create
        rospy.loginfo("POOL %s: state after addition: %s" % (self.viewport_name, self.browsers))
        return True

    def _update_browser(self, browser_pool_id, updated_browser):
        """
        Update existing browser instance
        """
        rospy.loginfo("POOL %s: state during updating: %s" % (self.viewport_name, self.browsers))
        current_browser = self.browsers[browser_pool_id]
        rospy.loginfo("Updating browser %s to it's new state: %s" % (current_browser, updated_browser))
        future_url = updated_browser.url
        future_geometry = WindowGeometry(x=updated_browser.geometry.x,
                                      y=updated_browser.geometry.y,
                                      width=updated_browser.geometry.width,
                                      height=updated_browser.geometry.height)

        current_geometry = current_browser.geometry

        if current_geometry != future_geometry:
            geom_success = self._update_browser_geometry(browser_pool_id, current_browser, future_geometry)
            if geom_success:
                rospy.loginfo("Successfully updated browser(%s) geometry from %s to %s" %  (browser_pool_id, current_geometry, future_geometry))
            else:
                rospy.logerr("Could not update geometry of browser (%s) (from %s to %s)" % (browser_pool_id, current_geometry, future_geometry))

        if current_browser.url != future_url:
            url_success = self._update_browser_url(browser_pool_id, current_browser, future_url)
            if url_success:
                rospy.loginfo("Successfuly updated browser(%s) url from %s to %s" % (browser_pool_id, current_browser.url, future_url))
            else:
                rospy.logerr("Could not update browser(%s) url from %s to %s" % (browser_pool_id, current_browser.url, future_url))

    def _update_browser_url(self, browser_pool_id, current_browser, future_url):
        try:
            rospy.loginfo("Updating URL of browser id %s from %s to %s" % (browser_pool_id, current_browser.url, future_url))
            current_browser.update_url(future_url)
            return True
        except Exception, e:
            rospy.logerr("Could not update url of browser id %s because: %s" % (browser_pool_id, e))
            return False


    def _update_browser_geometry(self, browser_pool_id, current_browser, future_geometry):
        try:
            current_browser.update_geometry(future_geometry)
            rospy.loginfo("Updated geometry of browser id %s" % browser_pool_id)
            return True
        except Exception, e:
            rospy.logerr("Could not update geometry of browser id %s because: %s" % (browser_pool_id, e))
            return False


    def handle_ros_message(self, data):
        """
        - if message has no AdhocBrowser messages in the array,
        shutdown all browsers
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
        for browser_pool_id in self._get_browsers_ids_to_remove(incoming_browsers_ids):
            rospy.loginfo("Removing browser id %s" % browser_pool_id)
            self._remove_browser(browser_pool_id)

        #create
        for browser_pool_id in self._get_browsers_ids_to_create(incoming_browsers_ids):
            rospy.loginfo("Creating browser with id %s" % browser_pool_id)
            self._create_browser(browser_pool_id, incoming_browsers[browser_pool_id])

        #update
        for browser_pool_id in self._get_browsers_ids_to_update(incoming_browsers_ids):
            rospy.loginfo("Updating browser with id %s" % browser_pool_id)
            self._update_browser(browser_pool_id, incoming_browsers[browser_pool_id])

        return True

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
