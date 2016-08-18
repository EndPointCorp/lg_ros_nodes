import rospy
import threading

from lg_common import ManagedAdhocBrowser
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
from lg_common.msg import BrowserExtension
from lg_common.msg import AdhocBrowser, AdhocBrowsers
from lg_common.helpers import get_app_instances_ids
from lg_common.srv import DirectorPoolQuery
from managed_browser import DEFAULT_BINARY
from urlparse import urlparse, parse_qs


class AdhocBrowserPool():
    """
    Handles browser pool on a biewport

    self.browsers ivar: dict of browser_pool_id and ManagedAdhocBrowser()

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
        self.browsers = {}
        self.extensions_root = "/opt/google/chrome/extensions/"
        self.lock = threading.Lock()
        self.viewport_name = viewport_name
        self._init_service()
        self.log_level = rospy.get_param('/logging/level', 0)

    def process_service_request(self, req):
        """
        Callback for service requests. We always return self.browsers
        """
        with self.lock:
            rospy.loginfo("Received DirectorPoolQuery service request" % self.viewport_name)
            return json.dumps(self.browsers, indent=4, sort_keys=True)

    def _init_service(self):
        service = rospy.Service('/browser_service/{}'.format(self.viewport_name),
                                DirectorPoolQuery,
                                self.process_service_request)

    def _get_incoming_browsers_dict(self, browsers):
        """
        Return dict(id: AdhocBrowser()) with all AdhocBrowser()s with pool ids as keys
        """
        return {b.id: b for b in browsers}

    def _remove_browser(self, browser_pool_id):
        """
        call .close() on browser object and cleanly delete the object
        """
        rospy.logdebug("Removing browser with id %s" % (self.viewport_name, browser_pool_id))
        self.browsers[browser_pool_id].close()
        del self.browsers[browser_pool_id]
        rospy.logdebug("State after %s removal: %s" % (self.viewport_name, browser_pool_id, self.browsers))

    def _filter_command_line_args(self, command_line_args):
        """
        remove/escape dangerous command_line_args
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

    def _get_browser_window_geometry(self, adhoc_browser_msg):
        """
        accepts AdhocBrowser message and returns WindowGeometry class instance
        """
        return WindowGeometry(x=adhoc_browser_msg.geometry.x,
                              y=adhoc_browser_msg.geometry.y,
                              width=adhoc_browser_msg.geometry.width,
                              height=adhoc_browser_msg.geometry.height)

    def _get_browser_extensions(self, adhoc_browser_msg):
        """
        accepts AdhocBrowser message and rewrites extension names
        to full extension's path under self.extensions_root that will
        be passed to --load-extension upon initialization

        returns: list
        """
        extensions = []
        for extension in adhoc_browser_msg.extensions:
            if extension.path:
                extensions.append(extension.path)
            else:
                extensions.append(self.extensions_root + extension.name)

        return extensions

    def _get_browser_command_line_args(self, adhoc_browser_msg):
        """
        accepts AdhocBrowser message and extracts command line arguments from it
        with some filtering of dangerous stuff

        returns: list
        """
        command_line_args = [cmd_arg.argument for cmd_arg in adhoc_browser_msg.command_line_args]
        return self._filter_command_line_args(command_line_args)

    def _get_browser_binary(self, adhoc_browser_msg):
        """
        accepts AdhocBrowser message and returns binary that should be used for
        launching ManagedAdhocBrowser

        returns: string
        """
        if adhoc_browser_msg.binary:
            binary = adhoc_browser_msg.binary
        else:
            binary = DEFAULT_BINARY
        return binary

    def _create_browser(self, new_browser_pool_id, new_browser, initial_state=None):
        """
        Accept AdhocBrowser message and an id.
        Assemble ManagedAdhocBrowser instance and add it to the pool

        Set the initial_state of it.

        returns: bool
        """
        geometry = self._get_browser_window_geometry(new_browser)
        extensions = self._get_browser_extensions(new_browser)
        command_line_args = self._get_browser_command_line_args(new_browser)
        binary = self._get_browser_binary(new_browser)
        url = self._add_ros_instance_url_param(new_browser.url, new_browser_pool_id)

        rospy.logdebug("Creating new browser %s with id %s and url %s" % (self.viewport_name, new_browser, new_browser_pool_id, url))
        managed_adhoc_browser = ManagedAdhocBrowser(geometry=geometry,
                                                    log_level=self.log_level,
                                                    slug=self.viewport_name + "_" + new_browser_pool_id,
                                                    user_agent=new_browser.user_agent,
                                                    extensions=extensions,
                                                    command_line_args=command_line_args,
                                                    binary=binary,
                                                    url=url,
                                                    uid=new_browser_pool_id,
                                                    scene_slug=new_browser.scene_slug
                                                    )

        self.browsers[new_browser_pool_id] = managed_adhoc_browser
        rospy.loginfo("State after addition of %s: %s" % (self.viewport_name, new_browser_pool_id, self.browsers.keys()))

        if initial_state:
            rospy.logdebug("Setting initial state of %s to %s" % (self.viewport_name, new_browser_pool_id, initial_state))
            browser_to_create.set_state(ApplicationState.STARTED)
        else:
            browser_to_create.set_state(ApplicationState.VISIBLE)

        return True

    def _hide_browsers(self, ids):
        """
        Accepts a list of browser pool ids to hide

        returns: bool
        """
        for browser_pool_id in ids:
            rospy.loginfo("Hiding browser with id %s" % browser_pool_id)
            self.browsers[browser_pool_id].set_state(ApplicationState.HIDDEN)

        return True

    def _destroy_browsers_ids(self, ids):
        """
        Accepts a list of browser ids to destroy (kill and remove)
        """
        rospy.loginfo("Browsers to remove = %s" % (self.viewport_name, ids))
        for browser_pool_id in ids:
            rospy.loginfo("Removing browser id %s" % browser_pool_id)
            self._remove_browser(browser_pool_id)

    def _get_all_preloadable_instances(self):
        """
        returns all browsers' ID prefixes shepherded by this pool, that are preloadable (which means
        that they are already shown as well as the instances that need to become unhidden)

        all browsers are identified with an ID. preloadable browsers have a prefix+suffix ID
        that's created from their normal stastic id + a suffix to make them be unique and reloaded
        upon every scene (unlike non-preloadable browsers that persists thru scenes)

        returns: list
        """

        # TODO (wz) - check if sha1 generates a `_` character
        preloadable_prefixes = [new_browser_id.split('_')[0] for new_browser_id in data.instances if '_' in new_browser_id]

        return preloadable_prefixes

    def unhide_browsers(self, data):
        """
        This is a callback executed upon a message that contains information
        about readiness of browsers that belong to a scene e.g.

        -----
          scene_slug: 659f6d8d-7c40-4f2c-911b-49390ac13e5f__tvn24
          activity_type: browser
          instances: ['50220834', '1234rewq']
        -----

        These browsers should already exist in STARTED state and this callback should
        unhide them and hide old browsers.

        """
        with self.lock():
            preloadable_prefixes = self._get_all_preloadable_instances()
            old_preloadable_instances_to_remove = self._get_old_preloadable_browser_instances(preloadable_prefixes)
            self._unhide_browser_instances(data)
            self._hide_browsers_ids(set(old_preloadable_instances_to_remove))
            self._destroy_browsers_ids(set(old_preloadable_instances_to_remove))

    def _get_old_preloadable_browser_instances(self, prelodable_prefixes):
        """
        Accepts a list of all preloadable prefixes
        Returns a list of browsers that
        """
        remove = []

        for browser in self.browsers.values():
            # all preloadable browsers from previous scene can be
            # safely marked for removal
            if browser.scene_slug != data.scene_slug:
                remove.append(browser.id)
            # edgcase coverage: if two consecutive scenes
            # have an identical slug then remove all preloadable
            # instances that are not in the incoming readiness message
            # use preloadable_prefixes to find which ones are to be removed
            if browser.id.split('_')[0] in preloadable_prefixes\
                and browser.id not in data.instances:
                remove.append(browser.id)

        return remove

    def _unhide_browser_instances(self, data):
        """
        Accepts a list of browser instances that are ready to be unhidden.
        Since data contains all browsers from a scene, we may not be shepherding
        some of the browsers on this viewport at this time.

        returns: bool
        """
        for browser_pool_id in data.instances:
            rospy.loginfo("Unhiding browser with id %s" % (browser_pool_id))
            try:
                self.browsers[browser_pool_id].set_state(ApplicationState.VISIBLE)
                return True
            except KeyError:
                rospy.logdebug("Could not remove %s from %s because browser doesnt exist in this pool" % (browser_pool_id, self.browsers.keys()))
                return False

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

    def handle_ros_message(self, data):
        """
        Handles a message that contains AdhocBrowser list inside of an AdhocBrowsers message.

        Creates and unhides browsers that are not preloadable.
        Creates browsers that need to preload.

        """

        with self.lock():
            self.scene_slug = data.scene_slug
            rospy.loginfo("============= NEW SCENE BEGIN (slug: %s) ===============" % self.scene_slug)
            incoming_browsers = self._get_incoming_browsers_dict(data.browsers)
            incoming_browsers_ids = set(incoming_browsers.keys())  # set
            current_browsers_ids = get_app_instances_ids(self.browsers)  # set
            ids_to_preload = set([incoming_browser_id for incoming_browser_id in incoming_browsers_ids if incoming_browsers[incoming_browser_id].preload])
            ids_to_remove = current_browsers_ids - incoming_browsers_ids
            ids_to_create = (incoming_browsers_ids - current_browsers_ids).union(ids_to_preload)
            rospy.loginfo('incoming ids: {}'.format(incoming_browsers_ids))
            rospy.loginfo('preloadable ids: {}'.format(ids_to_preload))
            rospy.loginfo('current ids: {}'.format(current_browsers_ids))
            rospy.loginfo('partitioned fresh ids: {}'.format(ids_to_create))
            rospy.loginfo('browsers to remove: {}'.format(ids_to_remove))
            self._create_browsers(ids_to_create, incoming_browsers)
            self._execute_browser_housekeeping(ids_to_create, ids_to_preload, ids_to_remove)
            rospy.loginfo("============== NEW SCENE END (slug: %s) ================" % self.scene_slug)
            return True

    def _execute_browser_housekeeping(self, ids_to_create, ids_to_preload, ids_to_remove):
        """
        Accepts ids to create, to preload and to remove to be able to decide
        which browsers to remove on the basis of their preloadability
        """
        # remove ids_to_remove but only if they constist of
        # non-preloadable browsers
        if len(ids_to_create - ids_to_preload) > 0:
            self._destroy_browsers_ids(ids_to_remove)

        # remove_all preloadable remnants
        if len(ids_to_create) == 0 and ids_to_remove:
            self._destroy_browsers_ids(ids_to_remove)

        return True

    def _create_browsers(self, ids_to_create, incoming_browsers):
        """
        Accepts a set of ids to create and incoming_browsers that contain
        browsers with these ids.
        """
        for browser_pool_id in ids_to_create:
            rospy.loginfo("Creating browser %s with preload flag: %s" % (browser_pool_id, incoming_browsers[browser_pool_id].preload))
            if incoming_browsers[browser_pool_id].preload:
                self._create_browser(browser_pool_id,
                                     incoming_browsers[browser_pool_id],
                                     ApplicationState.STARTED)
            else:
                self._create_browser(browser_pool_id,
                                     incoming_browsers[browser_pool_id])

        return True


    def handle_soft_relaunch(self, *args, **kwargs):
        current_browsers = self.browsers.keys()
        for browser_id in current_browsers:
            self._remove_browser(browser_id)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
