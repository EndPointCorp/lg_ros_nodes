import rospy
import threading
import json
import glob
import lg_common
import os
import time

from lg_common import ManagedAdhocBrowser, ManagedWindow
from lg_msg_defs.msg import ApplicationState
from lg_msg_defs.msg import WindowGeometry
from lg_msg_defs.msg import BrowserExtension
from lg_msg_defs.msg import AdhocBrowser, AdhocBrowsers, BrowserURL
from lg_common.helpers import get_app_instances_ids
from lg_msg_defs.srv import BrowserPool
from .managed_browser import DEFAULT_BINARY
from urllib.parse import urlparse, parse_qs, urlunparse
from urllib.parse import urlencode

from lg_common.logger import get_logger
logger = get_logger('adhoc_browser_pool')


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

    def __init__(
        self,
        viewport_name,
        extensions_root,
        hide_delay,
        destroy_delay,
    ):
        """
        AdhocBrowserPool manages a pool of browsers on one viewport.
        self.browsers ivar keeps a dict of (id: ManagedAdhocBrowser) per viewport.
        """

        self.browsers = {}
        self.browsers_info = {}
        self.hide_delay = hide_delay
        self.destroy_delay = destroy_delay

        # FIXME: Check that extensions_root ends with '/'
        self.extensions_root = extensions_root
        self.lg_common_internal_extensions_root = self._get_lg_common_extensions_root()
        self.lock = threading.Lock()
        self.viewport_name = viewport_name
        # FIXME: make this class ros - offline: move ros dependancies to script
        self._init_service()
        self.log_level = rospy.get_param('/logging/level', 0)
        self.rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
        self.rosbridge_secure = rospy.get_param('~rosbridge_secure', False)

    def _get_lg_common_extensions_root(self):
        """
        Gets path to a directory with extensions for lg_common
        e.g. '/opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages/lg_common' for
        deb distributed extensions or '/home/lg/catkin_ws/src/lg_common/src/lg_common'
        for lg_common built with catkin.

        It returns a first matched dir under which an `extensions` dir exists
        """
        logger.debug("Going to to check %s dirs looking for extensions" % lg_common.__path__)
        for module_directory in lg_common.__path__:
            possible_extensions_directory = module_directory + "/extensions"
            logger.debug("Checking if %s exists to load extensions from it" % possible_extensions_directory)
            if os.path.isdir(possible_extensions_directory):
                logger.debug("%s exists!" % possible_extensions_directory)
                return possible_extensions_directory

        return ''

    def _normalize_url(self, current_url, injected_get_args):
        replace = []

        url_parts = urlparse(current_url)
        if url_parts.query:
            get_args = parse_qs(url_parts.query, keep_blank_values=True)
            filtered = dict((k, v) for k, v in get_args.items() if k not in injected_get_args)

            newurl = urlunparse([
                url_parts.scheme, url_parts.netloc, url_parts.path, url_parts.params,
                urlencode(filtered, doseq=True),   # query string
                url_parts.fragment])

            return newurl

        return current_url

    def _serialize_browser_pool(self, with_info=False):
        """
        iterates over a dict of self.browsers and returns a dict
        where key is the ID of the browser and value is a json
        serialized representation of browsers' .__str__() method

        rtype: dict
        """
        serialized_browsers = {}
        for browser_id, browser in list(self.browsers.items()):
            serialized_browser = json.loads(browser.__str__())
            for key in list(self.browsers_info[browser_id].keys()):
                serialized_browser[key] = self.browsers_info[browser_id][key]

            serialized_browser['current_url_normalized'] = self._normalize_url(
                serialized_browser.get('current_url', ''),
                serialized_browser.get('injected_get_args', []))

            serialized_browsers[browser_id] = serialized_browser

        return serialized_browsers

    def process_service_request(self, req):
        """
        Callback for service requests. We always return self.browsers
        in a form of a strigified dictionary
        """

        with self.lock:
            response = json.dumps(self._serialize_browser_pool())
            logger.debug("Received BrowserPool service request")
            logger.debug("Returning %s" % response)

            try:
                options = json.loads(req.options)
                self._filter_service_response(options, response)
            except Exception:
                pass

            return response

    def _filter_service_response(self, options, response):
        pass

    def process_url_update(self, msg):
        with self.lock:
            browser_info = self.browsers_info.get(msg.browser_id, None)
            if browser_info:
                browser_info['current_url'] = msg.url
                browser_info['url_ts'] = time.time()

    def _init_service(self):
        rospy.Service('/browser_service/{}'.format(self.viewport_name),
                      BrowserPool,
                      self.process_service_request)

        rospy.Subscriber("/browser_service/{}/update_url".format(self.viewport_name),
                         BrowserURL,
                         self.process_url_update)

    def _get_incoming_browsers_dict(self, browsers):
        """
        Return dict(id: AdhocBrowser()) with all AdhocBrowser()s with pool ids as keys
        """
        return {b.id: b for b in browsers}

    def _remove_browser(self, browser_pool_id):
        """
        call .close() on browser object and cleanly delete the object
        """
        logger.debug("Removing browser with id %s" % (browser_pool_id))
        self.browsers[browser_pool_id].close()
        del self.browsers[browser_pool_id]
        del self.browsers_info[browser_pool_id]
        logger.debug("State after %s removal: %s" % (browser_pool_id, self.browsers))

    def _filter_command_line_args(self, command_line_args):
        """
        remove/escape dangerous command_line_args
        """
        result = []
        for arg in command_line_args:
            if ';' in arg:
                logger.error("There is ';' in command line arguments for adhock browser")
                return []

            if 'enable-arc' in arg or 'enable-nacl' in arg:
                logger.error("Unsupported command line arg %s" % arg)
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

    def _get_browser_extensions(self, adhoc_browser_msg, system_extensions):
        """
        accepts AdhocBrowser message and rewrites extension names
        to full extension's path under self.extensions_root that will
        be passed to --load-extension upon initialization

        adds extensions from system_extensions which were added by
        pool itself to serve internal needs

        if extensions exists under lg_common/extensions directory then
        it's going have higher prio over external extensions. In other words
        choose lg_ros_nodes shipped extensions over external extensions from
        the filesystem located under self.extensions_root

        returns: list of extensions paths
        """
        extensions = []
        msg_extensions = [extension.name for extension in adhoc_browser_msg.extensions]
        for extension in system_extensions + msg_extensions:
            if os.path.isdir(self.lg_common_internal_extensions_root + "/" + extension):
                extensions.append(self.lg_common_internal_extensions_root + "/" + extension)
            else:
                extensions.append(self.extensions_root + extension)

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
        if adhoc_browser_msg.version:
            binary = "/usr/bin/google-chrome-%s" % adhoc_browser_msg.version
        else:
            binary = DEFAULT_BINARY
        return binary

    def _get_browser_allowed_urls(self, adhoc_browser_msg):
        if adhoc_browser_msg.allowed_urls:
            return adhoc_browser_msg.allowed_urls

        return None

    def _create_browser(self, new_browser_pool_id, new_browser, initial_state=None):
        """
        Accept AdhocBrowser message and an id.
        Assemble ManagedAdhocBrowser instance and add it to the pool

        Set the initial_state of it.

        returns: bool
        """
        browser_info = {}
        browser_info['ros_instance_name'] = new_browser_pool_id
        browser_info['initial_url'] = new_browser.url
        browser_info['current_url'] = new_browser.url
        browser_info['url_ts'] = time.time()
        browser_info['creation_ts'] = time.time()
        browser_info['injected_get_args'] = []

        additional_extensions = []
        if new_browser.allowed_urls:
            additional_extensions.append('monitor_page_urls')

        if new_browser.preload:
            additional_extensions.append('ros_window_ready')

        additional_extensions.append('current_url')

        geometry = self._get_browser_window_geometry(new_browser)
        extensions = self._get_browser_extensions(new_browser, additional_extensions)
        command_line_args = self._get_browser_command_line_args(new_browser)
        default_args_removal = [cmd_arg.argument for cmd_arg in new_browser.default_args_removal]
        binary = self._get_browser_binary(new_browser)

        if new_browser.custom_preload_event:
            logger.debug("Using custom preloading event")
            new_browser.url = self._inject_get_argument(new_browser.url, 'use_app_event', 1)
        else:
            logger.debug("NOT Using custom preloading event")

        # Add ros_instance_name to evry browser
        new_browser.url = self._inject_get_argument(new_browser.url, 'ros_instance_name', new_browser_pool_id)
        browser_info['injected_get_args'].append('ros_instance_name')

        # Add viewport to evry browser
        new_browser.url = self._inject_get_argument(new_browser.url, 'viewport', self.viewport_name)
        browser_info['injected_get_args'].append('viewport')

        # Default host for rosbridge is localhost
        # and we are not going to change that very often
        new_browser.url = self._inject_get_argument(new_browser.url,
                                                    'rosbridge_port',
                                                    self.rosbridge_port)
        browser_info['injected_get_args'].append('rosbridge_port')

        new_browser.url = self._inject_get_argument(new_browser.url,
                                                    'rosbridge_secure',
                                                    1 if self.rosbridge_secure else 0)
        browser_info['injected_get_args'].append('rosbridge_secure')

        allowed_urls = self._get_browser_allowed_urls(new_browser)

        if allowed_urls:
            new_browser.url = self._inject_get_argument(
                new_browser.url,
                'allowed_urls',
                allowed_urls
            )
            browser_info['injected_get_args'].append('allowed_urls')

        logger.debug(
            "Creating new browser %s with id %s and url %s" %
            (new_browser, new_browser_pool_id, new_browser.url)
        )
        managed_adhoc_browser = ManagedAdhocBrowser(geometry=geometry,
                                                    log_level=self.log_level,
                                                    slug=self.viewport_name + "_" + new_browser_pool_id,
                                                    user_agent=new_browser.user_agent,
                                                    extensions=extensions,
                                                    command_line_args=command_line_args,
                                                    default_args_removal=default_args_removal,
                                                    binary=binary,
                                                    url=new_browser.url,
                                                    uid=new_browser_pool_id,
                                                    scene_slug=new_browser.scene_slug,
                                                    preload=new_browser.preload,
                                                    user_data_dir=new_browser.user_data_dir,
                                                    kiosk=new_browser.kiosk,
                                                    layer=ManagedWindow.LAYER_ABOVE,
                                                    )

        self.browsers[new_browser_pool_id] = managed_adhoc_browser
        self.browsers_info[new_browser_pool_id] = browser_info
        logger.debug("State after addition of %s: %s" % (new_browser_pool_id, list(self.browsers.keys())))

        if initial_state:
            logger.debug("Setting initial state of %s to %s" % (new_browser_pool_id, initial_state))
            managed_adhoc_browser.set_state(ApplicationState.STARTED)
        else:
            managed_adhoc_browser.set_state(ApplicationState.VISIBLE)

        return True

    def minimize_browsers(self, ids):
        """
        For now, just hide the browserts
        """
        self._hide_browsers_ids(ids)

    def _hide_browsers_ids(self, ids):
        """
        Accepts a list of browser pool ids to hide

        returns: bool
        """
        for browser_pool_id in ids:
            logger.debug("Hiding browser with id %s" % browser_pool_id)
            self.browsers[browser_pool_id].set_state(ApplicationState.HIDDEN)

        return True

    def _destroy_browsers_ids(self, ids):
        """
        Accepts a list of browser ids to destroy (kill and remove)
        """
        logger.debug("Browsers to remove = %s" % (ids))
        for browser_pool_id in ids:
            logger.debug("Removing browser id %s" % browser_pool_id)
            self._remove_browser(browser_pool_id)

    def _get_all_preloadable_instances(self, data):
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
        with self.lock:
            logger.debug("============UNHIDING BEGIN %s ============" % data.instances)
            preloadable_prefixes = self._get_all_preloadable_instances(data)
            logger.debug("Preloadable prefixes: %s" % preloadable_prefixes)
            old_preloadable_instances_to_remove = frozenset(self._get_old_preloadable_browser_instances(preloadable_prefixes, data))
            self._unhide_browser_instances(data)
            logger.debug("Old preloadable instances to remove: %s" % old_preloadable_instances_to_remove)
            rospy.sleep(self.hide_delay)
            self._hide_browsers_ids(old_preloadable_instances_to_remove)
            rospy.sleep(self.destroy_delay)
            self._destroy_browsers_ids(old_preloadable_instances_to_remove)
            logger.debug("============UNHIDING END %s   ============" % data.instances)

    def _get_old_preloadable_browser_instances(self, preloadable_prefixes, data):
        """
        Accepts a list of all preloadable prefixes
        Returns a list of prelodable browser prefixes that should be removed

        Consider preloadable browsers only
        """
        remove = []

        for browser_instance in list(self.browsers.values()):
            # consider only preloadable browsers
            if browser_instance.preload is True:
                # if browser prefix is not mentioned in the incoming scene
                # then it may be marked for removal
                if browser_instance.id.split('_')[0] not in preloadable_prefixes:
                    remove.append(browser_instance.id)
                # edgcase coverage: if two consecutive scenes
                # have an identical slug then remove all preloadable
                # instances that are not in the incoming readiness message
                # use preloadable_prefixes to find which ones are to be removed
                if browser_instance.id.split('_')[0] in preloadable_prefixes and browser_instance.id not in data.instances:
                    remove.append(browser_instance.id)

        return remove

    def _unhide_browser_instances(self, data):
        """
        Accepts a list of browser instances that are ready to be unhidden.
        Since data contains all browsers from a scene, we may not be shepherding
        some of the browsers on this viewport at this time.

        returns: bool
        """
        for browser_pool_id in data.instances:
            logger.debug("Unhiding browser with id %s" % (browser_pool_id))
            try:
                self.browsers[browser_pool_id].set_state(ApplicationState.VISIBLE)
            except KeyError:
                logger.debug("Could not remove %s from %s because browser doesnt exist in this pool" % (browser_pool_id, list(self.browsers.keys())))
            except Exception as e:
                logger.debug("Could not remove %s from %s because browser doesnt exist in this pool because: %s" % (browser_pool_id, list(self.browsers.keys()), e))

    def _inject_get_argument(self, url, get_arg_name, get_arg_value):
        """
        Accepts a string of url and injects get argument with
        get_arg_name and get_arg_value

        returns modified URL

        """
        url_parts = urlparse(url)

        if url_parts.query is None:
            new_q = '{}={}'.format(get_arg_name, get_arg_value)
            url_parts = url_parts._replace(query=new_q)
            return url_parts.geturl()

        get_args = parse_qs(url_parts.query)
        get_args[get_arg_name] = get_arg_value

        # join args without encoding - browser will do the rest
        arg_list = []
        for item in list(get_args.items()):
            if type(item[1]) == list:
                for val in item[1]:
                    arg = str(item[0]) + "=" + str(val)
                    arg_list.append(arg)
            else:
                arg = str(item[0]) + "=" + str(item[1])
                arg_list.append(arg)

        new_q = '&'.join(arg_list)
        url_parts = url_parts._replace(query=new_q)
        return url_parts.geturl()

    def handle_ros_message(self, data):
        """
        Handles a message that contains AdhocBrowser list inside of an AdhocBrowsers message.

        Creates and unhides browsers that are not preloadable.
        Creates browsers that need to preload.

        """

        with self.lock:
            self.scene_slug = data.scene_slug
            logger.debug("============= NEW SCENE BEGIN (slug: %s) ===============" % self.scene_slug)
            incoming_browsers = self._get_incoming_browsers_dict(data.browsers)
            incoming_browsers_ids = set(incoming_browsers.keys())  # set
            current_browsers_ids = get_app_instances_ids(self.browsers)  # set
            ids_to_preload = set([incoming_browser_id for incoming_browser_id in incoming_browsers_ids if incoming_browsers[incoming_browser_id].preload])
            ids_to_remove = current_browsers_ids - incoming_browsers_ids
            ids_to_create = (incoming_browsers_ids - current_browsers_ids).union(ids_to_preload)
            logger.debug('incoming ids: {}'.format(incoming_browsers_ids))
            logger.debug('preloadable ids: {}'.format(ids_to_preload))
            logger.debug('current ids: {}'.format(current_browsers_ids))
            logger.debug('partitioned fresh ids: {}'.format(ids_to_create))
            logger.debug('browsers to remove: {}'.format(ids_to_remove))
            self._create_browsers(ids_to_create, incoming_browsers)
            self._execute_browser_housekeeping(ids_to_create, ids_to_preload, ids_to_remove)
            logger.debug("============== NEW SCENE END (slug: %s) ================" % self.scene_slug)
            return True

    def _execute_browser_housekeeping(self, ids_to_create, ids_to_preload, ids_to_remove):
        """
        Accepts ids to create, to preload and to remove to be able to decide
        which browsers to remove on the basis of their preloadability

        Remove all browsers that are not preloadable immediately
        """
        if len(ids_to_preload) > 0:
            # if there exist ANY preloadable browsers in new scene then
            # filter out all old browsers that are preloadable and need to stay
            # on the screens until new browsers become ready and old browsers
            # will get hidden and removed
            ids_to_remove = [browser for browser in ids_to_remove if '_' not in browser]

        self._destroy_browsers_ids(ids_to_remove)

        return True

    def _create_browsers(self, ids_to_create, incoming_browsers):
        """
        Accepts a set of ids to create and incoming_browsers that contain
        browsers with these ids.
        """
        for browser_pool_id in ids_to_create:
            logger.debug("Creating browser %s with preload flag: %s" % (browser_pool_id, incoming_browsers[browser_pool_id].preload))
            if incoming_browsers[browser_pool_id].preload:
                self._create_browser(browser_pool_id,
                                     incoming_browsers[browser_pool_id],
                                     ApplicationState.STARTED)
            else:
                self._create_browser(browser_pool_id,
                                     incoming_browsers[browser_pool_id])

        return True

    def handle_soft_relaunch(self, *args, **kwargs):
        # iterate over all browsers and call the handle_soft_relaunch
        # from the great grand parent ManagedApplication
        for browser in self.browsers:
            self.browsers[browser].handle_soft_relaunch()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
