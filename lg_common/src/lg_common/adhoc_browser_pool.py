import rospy
import threading
import urllib

from lg_common import ManagedAdhocBrowser
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
from lg_common.msg import AdhocBrowser, AdhocBrowsers
from lg_common.helpers import get_app_instances_to_manage
from lg_common.helpers import get_app_instances_ids
from lg_common.srv import DirectorPoolQuery

from urlparse import urlparse, parse_qs, parse_qsl


class AdhocBrowserPool():
    """
    Handles browser pool in self.browsers
    Dict(id => ManagedAdhocBrowser)
    """
    def __init__(self, viewport_name):
        """
        AdhocBrowserPool manages a pool of browsers on one viewport.
        self.browsers ivar keeps a dict of (id: ManagedAdhocBrowser) per viewport.
        """
        self.viewport_name = viewport_name
        self._init_service()
        self.browsers = {}
        self.log_level = rospy.get_param('/logging/level', 0)
        self.type_key = "adhoc"

    def process_service_request(self, req):
        """
        Callback for service requests. We always return self.browsers
        """
        rospy.loginfo("POOL %s: Received Query service" % self.viewport_name)
        return str(self.browsers)

    def _init_service(self):
        service = rospy.Service('/browser_service/{}'.format(self.viewport_name),
                                DirectorPoolQuery,
                                self.process_service_request)
        return service

    def _unpack_incoming_browsers(self, browsers):
        """
        Return dict(id: AdhocBrowser) with all AdhocBrowsers with ids as keys
        """
        return {b.id: b for b in browsers}

    def _remove_browser(self, browser_pool_id):
        """
        call .close() on browser object and cleanly delete the object
        """
        rospy.loginfo("POOL %s: Removing browser with id %s" % (self.viewport_name, browser_pool_id))
        self.browsers[browser_pool_id].close()
        del self.browsers[browser_pool_id]
        rospy.loginfo("POOL %s: state after %s removal: %s" % (self.viewport_name, browser_pool_id, self.browsers))

    def _create_browser(self, new_browser_pool_id, new_browser):
        """
        Create new browser instance with desired geometry.
        """
        geometry = WindowGeometry(x=new_browser.geometry.x,
                                  y=new_browser.geometry.y,
                                  width=new_browser.geometry.width,
                                  height=new_browser.geometry.height)

        ros_instance_id = self._get_ros_instance_id(new_browser_pool_id)
        rospy.loginfo("Browser URL before ros_instance addition: %s" % new_browser.url)
        new_url = self._add_ros_instance_url_param(new_browser.url, ros_instance_id)
        rospy.loginfo("Browser URL after ros_instance addition: %s" % new_url)

        browser_to_create = ManagedAdhocBrowser(geometry=geometry,
                                                log_level=self.log_level,
                                                slug=self.viewport_name + "_" + new_browser_pool_id,
                                                url=new_url)

        browser_to_create.set_state(ApplicationState.VISIBLE)
        rospy.loginfo("POOL %s: Creating new browser %s with id %s and url %s" % (self.viewport_name, new_browser, new_browser_pool_id, new_url))
        self.browsers[new_browser_pool_id] = browser_to_create
        rospy.loginfo("POOL %s: state after addition of %s: %s" % (self.viewport_name, new_browser_pool_id, self.browsers))
        return True

    def _add_ros_instance_url_param(self, url, ros_instance_name):
        """
        Accepts strings of url and ros_instance_name
        Injects ros_instance_name as a first GET argument
        Returns modified URL
        """
        url_parts = urlparse(url)

        if url_parts.query is None:
            new_q = 'ros_instance_name={}'.format(ros_instance_name)
            url_parts = url_parts._replace(query=new_q)
            return url_parts.geturl()

        get_args = parse_qs(url_parts.query)
        get_args['ros_instance_name'] = ros_instance_name

        # join args without encoding - browser will do the rest
        new_q = '&'.join((['='.join([str(item[0]),str(item[1][0])]) for item in get_args.items()]))
        #new_q = urllib.urlencode(get_args, doseq=True)
        url_parts = url_parts._replace(query=new_q)
        return url_parts.geturl()

    def _get_ros_instance_id(self, new_browser_pool_id):
        return "%s__%s__%s" % (self.type_key, self.viewport_name, new_browser_pool_id)

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

        incoming_browsers = self._unpack_incoming_browsers(data.browsers)
        incoming_browsers_ids = set(incoming_browsers.keys())  # set
        current_browsers_ids = get_app_instances_ids(self.browsers)  # set

        for browser_pool_id in current_browsers_ids:
            rospy.loginfo("Removing browser id %s" % browser_pool_id)
            self._remove_browser(browser_pool_id)

        for browser_pool_id in incoming_browsers_ids:
            rospy.loginfo("Creating browser with id %s" % browser_pool_id)
            self._create_browser(browser_pool_id, incoming_browsers[browser_pool_id])

        return True

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
