import rospy
import threading
import urllib
import re

from lg_common import ManagedAdhocBrowser
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
from lg_common.msg import BrowserExtension
from lg_common.msg import AdhocBrowser, AdhocBrowsers
from lg_common.helpers import get_app_instances_ids
from lg_common.helpers import url_compare
from lg_common.helpers import geometry_compare
from lg_common.helpers import browser_eligible_for_reuse
from lg_common.srv import DirectorPoolQuery
from managed_browser import DEFAULT_BINARY
from urlparse import urlparse, parse_qs, parse_qsl


class AdhocBrowserPool():
    """
    Handles browser pool in self.browsers

    self.browsers ivar is a dict where keys are browser_pool_id
    and values are ManagedAdhocBrowser instances.

    Smooth transitions functionality overview:
    - handle_ros_message creates new browsers
    - wait for ready signal (see rediness.py)
    - when new browsers ready, hide and destroy
      old browsers and show new

    """
    def __init__(self, viewport_name):
        """
        AdhocBrowserPool manages a pool of browsers on one viewport.
        self.browsers ivar keeps a dict of (id: ManagedAdhocBrowser) per viewport.
        """
        self.extensions_root = "/opt/google/chrome/extensions/"
        self.lock = threading.Lock()
        self.viewport_name = viewport_name
        self._init_service()
        self.browsers = {}
        self.log_level = rospy.get_param('/logging/level', 0)
        self.type_key = "adhoc"

    def process_service_request(self, req):
        """
        Callback for service requests. We always return self.browsers
        """
        with self.lock:
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

    def _partition_existing_browser_ids(self, incoming_browsers):
        """
        Determine which incoming browsers are already being shown.
        Take their URL *and* geometry/placement into consideration

        Return set of ids for:
         - browsers that need to be created (fresh_ids)
         - browsers that already exist (existing_ids)
        """
        def strip_ros_instance_name(url):
            stripped_url = re.sub(r'[&?]ros_instance_name=[^&]*', '', url)
            return stripped_url

        def incoming_browser_already_exists_in_the_pool(checked_browser):
            for browser in self.browsers.values():
                if browsers_are_identical(checked_browser, browser):
                    return True
            return False

        def existing_browser_exists_in_incoming_browsers(checked_browser):
            for browser in incoming_browsers:
                if browsers_are_identical(checked_browser, browser):
                    return True
            return False

        def browsers_are_identical(browser_a, browser_b):
            # go thru self.browsers and match url + geom
            browser_a_url = strip_ros_instance_name(browser_a.url)
            browser_b_url = strip_ros_instance_name(browser_b.url)
            rospy.logdebug("Comparison: %s (%s) vs %s (%s)" % (browser_a_url,
                                                               browser_a.id,
                                                               browser_b_url,
                                                               browser_b.id
                                                               ))
            if browser_a_url == browser_b_url:
                if geometry_compare(browser_a, browser_b):
                    return True
            return False

        # incoming_browsers_that_should_not_be_created = [b.id for b in incoming_browsers if incoming_browser_already_exists_in_the_pool(b)]
        incoming_browsers_that_should_be_created = [b.id for b in incoming_browsers if not incoming_browser_already_exists_in_the_pool(b)]
        existing_browsers_to_remove = [b.id for b in self.browsers.values() if not existing_browser_exists_in_incoming_browsers(b)]
        # existing_browsers_to_leave = self.browsers.keys() - existing_browsers_to_remove
        return existing_browsers_to_remove, incoming_browsers_that_should_be_created

    def _remove_browser(self, browser_pool_id):
        """
        call .close() on browser object and cleanly delete the object
        """
        rospy.logdebug("POOL %s: Removing browser with id %s" % (self.viewport_name, browser_pool_id))
        self.browsers[browser_pool_id].close()
        del self.browsers[browser_pool_id]
        rospy.logdebug("POOL %s: state after %s removal: %s" % (self.viewport_name, browser_pool_id, self.browsers))

    def _filter_command_line_args(self, command_line_args):
        """
        Remove/escape dangerous command_line_args
        """
        result = []
        for arg in command_line_args:
            if ';' in arg:
                rospy.logerror("There is ';' in command line arguments for adhock browser")
                return []

            if 'enable-arc' in arg or 'enable-nacl' in arg:
                rospy.logerror("Unsupported command line arg %s" % arg)
                return []

            result.append(arg)

        return result

    def _create_browser(self, new_browser_pool_id, new_browser, initial_state=None):
        """
        Create new browser instance with desired geometry.
        """
        geometry = WindowGeometry(x=new_browser.geometry.x,
                                  y=new_browser.geometry.y,
                                  width=new_browser.geometry.width,
                                  height=new_browser.geometry.height)

        extensions = []
        for extension in new_browser.extensions:
            if extension.path:
                extensions.append(extension.path)
            else:
                extensions.append(self.extensions_root + extension.name)

        command_line_args = [cmd_arg.argument for cmd_arg in new_browser.command_line_args]

        command_line_args = self._filter_command_line_args(command_line_args)

        if new_browser.binary:
            binary = new_browser.binary
        else:
            binary = DEFAULT_BINARY

        ros_instance_id = self._get_ros_instance_id(new_browser_pool_id)
        rospy.loginfo("Browser URL before ros_instance addition: %s" % new_browser.url)
        new_url = self._add_ros_instance_url_param(new_browser.url, ros_instance_id)
        rospy.loginfo("Browser URL after ros_instance addition: %s" % new_url)

        browser_to_create = ManagedAdhocBrowser(geometry=geometry,
                                                log_level=self.log_level,
                                                slug=self.viewport_name + "_" + new_browser_pool_id,
                                                user_agent=new_browser.user_agent,
                                                extensions=extensions,
                                                command_line_args=command_line_args,
                                                binary=binary,
                                                url=new_url,
                                                uid=new_browser_pool_id
                                                )

        rospy.logdebug("POOL %s: Creating new browser %s with id %s and url %s" % (self.viewport_name, new_browser, new_browser_pool_id, new_url))
        self.browsers[new_browser_pool_id] = browser_to_create
        rospy.loginfo("POOL %s: state after addition of %s: %s" % (self.viewport_name, new_browser_pool_id, self.browsers))

        if initial_state:
            rospy.loginfo("POOL %s: setting initial state of %s to %s" % (self.viewport_name, new_browser_pool_id, initial_state))
            browser_to_create.set_state(ApplicationState.STARTED)
        else:
            browser_to_create.set_state(ApplicationState.VISIBLE)

        return True

    def _hide_browsers(self, ids):
        """
        Hide old browsers
        """
        for browser_pool_id in ids:
            rospy.loginfo("Hiding browser with id %s" % browser_pool_id)
            self.browsers[browser_pool_id].set_state(ApplicationState.HIDDEN)

    def _destroy_browsers(self, ids):
        """
        Destroy browsers instances,
        free system resources
        """
        rospy.loginfo("POOL %s: browsers to remove = %s" % (self.viewport_name, ids))
        for browser_pool_id in ids:
            rospy.loginfo("Removing browser id %s" % browser_pool_id)
            self._remove_browser(browser_pool_id)

    def unhide_browsers(self, data):
        """
        Listen on a topic that carries following Ready type:
        -----
          scene_slug: 659f6d8d-7c40-4f2c-911b-49390ac13e5f__tvn24
          activity_type: browser
          instances: ['50220834']
        -----

        Activate browser instances mentioned in 'instances' that belong to 'scene_slug'
        and remove old browsers
        """
        if data.scene_slug == self.last_scene_slug:
            for browser_pool_id in data.instances:
                rospy.loginfo("Unhiding browser with id %s (type of id: %s)" % (browser_pool_id, type(browser_pool_id)))
                rospy.loginfo("State of the pool before set visible: %s" % self.browsers)
                try:
                    self.browsers[browser_pool_id].set_state(ApplicationState.VISIBLE)
                except KeyError:
                    rospy.loginfo("Could not remove %s from %s because of KeyError" % (browser_pool_id, self.browsers))

            # For now there is no scene_slug => browsers[] tracking
            # so all the browsers who not mentioned in instances
            # treated as old and should be hided.

            old_browsers = set(self.browsers.keys()) - set(data.instances)
            rospy.loginfo("State before hiding browsers %s is %s" % (old_browsers, self.browsers))
            self._hide_browsers(old_browsers)
            rospy.loginfo("State after hiding browsers %s is %s" % (old_browsers, self.browsers))
            self._destroy_browsers(old_browsers)
            rospy.loginfo("State after destroying browsers %s is %s" % (old_browsers, self.browsers))
        else:
            # That's possible if we've got new scene, before
            # the old scene was activated
            # (before the browsers for old scene become ready).
            #
            # to handle this sittuation properly we need to remove all old instances

            self._hide_browsers(set(self.browsers.keys()))
            self._destroy_browsers(set(self.browsers.keys()))

    def _update_browser_url(self, browser_pool_id, current_browser, future_url):
        try:
            rospy.logdebug("Updating URL of browser id %s from %s to %s" % (browser_pool_id, current_browser.url, future_url))
            future_url = self._add_ros_instance_url_param(future_url, self._get_ros_instance_id(browser_pool_id))
            current_browser.update_url(future_url)
            return True
        except Exception, e:
            rospy.logerr("Could not update url of browser id %s because: %s" % (browser_pool_id, e))
            return False

    def _update_browser_geometry(self, browser_pool_id, current_browser, future_geometry):
        try:
            current_browser.update_geometry(future_geometry)
            rospy.logdebug("Updated geometry of browser id %s" % browser_pool_id)
            return True
        except Exception, e:
            rospy.logerr("Could not update geometry of browser id %s because: %s" % (browser_pool_id, e))
            return False

    def _partition_existing_browser_ids(self, incoming_browsers):
        """
        Determine which incoming browsers are already being shown.
        Take their URL *and* geometry/placement into consideration
        Return set of ids for:
         - browsers that need to be created (fresh_ids)
         - browsers that already exist (existing_ids)
        """
        def strip_ros_instance_name(url):
            stripped_url = re.sub(r'[&?]ros_instance_name=[^&]*', '', url)
            return stripped_url

        def browser_exists(incoming_browser):
            # go thru self.browsers and match url + geom
            for existing_browser in self.browsers.values():
                existing_browser_url = strip_ros_instance_name(existing_browser.url)
                incoming_browser_url = strip_ros_instance_name(incoming_browser.url)
                rospy.loginfo("Comparison: %s vs %s" % (existing_browser_url, incoming_browser_url))
                if existing_browser_url == incoming_browser_url:
                    rospy.loginfo("incoming_browser's url %s matches existing browser %s" % (incoming_browser_url, existing_browser_url))
                    if geometry_compare(incoming_browser, existing_browser):
                        rospy.loginfo("OK === incoming_browser's geometry matches existing browser ===")
                        return True
                    else:
                        rospy.loginfo("incoming_browser's geometry %s does not matche existing browser %s" % (incoming_browser.geometry, existing_browser.geometry))
                        return False

                else:
                    rospy.loginfo("incoming_browser: %s does not match existing browser %s" % (incoming_browser, existing_browser))
                    return False
            return False

        existing_ids = [b.id for b in incoming_browsers if browser_exists(b)]
        fresh_ids = [b.id for b in incoming_browsers if not browser_exists(b)]

        return existing_ids, fresh_ids

    def _update_browser(self, browser_pool_id, updated_browser):
        """
        Update existing browser instance
        Accept existing browser_pool_id and incoming browser
        """
        rospy.logdebug("POOL %s: state during updating: %s" % (self.viewport_name, self.browsers))
        current_browser = self.browsers[browser_pool_id]
        rospy.logdebug("Updating browser %s to it's new state: %s" % (current_browser, updated_browser))
        future_url = updated_browser.url
        future_geometry = WindowGeometry(x=updated_browser.geometry.x,
                                         y=updated_browser.geometry.y,
                                         width=updated_browser.geometry.width,
                                         height=updated_browser.geometry.height)

        current_geometry = current_browser.geometry

        if current_browser.url != future_url:
            self._update_browser_url(browser_pool_id, current_browser, future_url)
        else:
            rospy.logdebug("POOL %s: not updating url of browser %s (old url=%s, new url=%s)" %
                           (self.viewport_name, current_browser, current_browser.url, future_url))

        geom_success = self._update_browser_geometry(browser_pool_id, current_browser, future_geometry)

        if geom_success:
            rospy.logdebug("Successfully updated browser(%s) geometry from %s to %s" % (browser_pool_id, current_geometry, future_geometry))
        else:
            rospy.logerr("Could not update geometry of browser (%s) (from %s to %s)" % (browser_pool_id, current_geometry, future_geometry))

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
        get_args['ros_instance_name'] = [ros_instance_name]

        # join args without encoding - browser will do the rest
        new_q = '&'.join((['='.join([str(item[0]), str(item[1][0])]) for item in get_args.items()]))
        url_parts = url_parts._replace(query=new_q)
        return url_parts.geturl()

    def _get_ros_instance_id(self, new_browser_pool_id):
        # return "%s__%s__%s" % (self.type_key, self.viewport_name, new_browser_pool_id)
        return new_browser_pool_id

    def handle_ros_message(self, data):
        """
        - if message has no AdhocBrowser messages in the array,
        shutdown all browsers
        - if the message has AdhocBrowser messages in the array:
        -- check for an existing browser with the same id.
        --- if it exists, don't update - always create new one
        --- if no existing browser, create one.
        --- if an existing browser has an id that doesn't match the new message, then
            shut it down and remove it from the tracked instances.

        Arguments:
            data: AdhocBrowsers, which is an array of AdhocBrowser

        AdhocBrowsers
            lg_common/AdhocBrowser[] browsers
            string scene_slug

        AdhocBrowser
            string id
            string url
            string user_agent
            string binary
            lg_common/WindowGeometry geometry
            lg_common/BrowserExtension[] extensions
            lg_common/BrowserCmdArg[] command_line_args

        Edgecase for browser persistence:

            msg1:
                adhoc_0 -> wp.pl *
                adhoc_1 -> onet.pl
                adhoc_2 -> endpoint.com


            msg2:
                adhoc_0 -> foo.com
                adhoc_1 -> wp.pl *
                adhoc_2 -> bar
            [*] should persist

            existing ids: adhoc_0
            fresh ids: adhoc_0, adhoc_2

            adhoc_0 is existing and fresh in the same time

        """
        rospy.loginfo("================ NEW SCENE ===============")

        # Do we wait for all browsers instance ready or not

        rospy.loginfo("============= NEW SCENE ===============")
        # keep the last scene slug for case when
        # someone decides to send a message with preload set to true
        # to be able to differentiate between previous
        # scene and incoming scene instances
        self.last_scene_slug = data.scene_slug

        incoming_browsers = self._unpack_incoming_browsers(data.browsers)
        incoming_browsers_ids = set(incoming_browsers.keys())  # set
        current_browsers_ids = get_app_instances_ids(self.browsers)  # set
        ids_to_preload = set([incoming_browser_id for incoming_browser_id in incoming_browsers_ids if incoming_browsers[incoming_browser_id].preload])
        ids_to_remove = incoming_browsers_ids - current_browsers_ids
        ids_to_create = current_browsers_ids - incoming_browsers_ids + ids_to_preload

        rospy.loginfo('incoming ids: {}'.format(incoming_browsers_ids))
        rospy.loginfo('preloadable ids: {}'.format(ids_to_preload))
        rospy.loginfo('current ids: {}'.format(current_browsers_ids))
        rospy.loginfo('partitioned fresh ids: {}'.format(ids_to_create))
        rospy.loginfo('browsers to remove: {}'.format(ids_to_remove))

        for browser_pool_id in ids_to_create:
            rospy.loginfo("Creating browser %s with preload flag: %s" % (browser_pool_id, incoming_browsers[browser_pool_id].preload))
            if incoming_browsers[browser_pool_id].preload:
                self._create_browser(browser_pool_id,
                                     incoming_browsers[browser_pool_id],
                                     ApplicationState.STARTED)
            else:
                self._create_browser(browser_pool_id,
                                     incoming_browsers[browser_pool_id])


        for browser_pool_id in ids_to_remove:
            rospy.loginfo("Removing browser id %s" % browser_pool_id)
            self._remove_browser(browser_pool_id)

        return True

    def handle_soft_relaunch(self, *args, **kwargs):
        current_browsers = self.browsers.keys()
        for browser_id in current_browsers:
            self._remove_browser(browser_id)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
