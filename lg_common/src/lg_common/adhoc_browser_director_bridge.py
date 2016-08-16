import rospy
import uuid
import json

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from lg_common.msg import BrowserExtension
from interactivespaces_msgs.msg import GenericMessage
from lg_common import ManagedWindow
from lg_common.msg import WindowGeometry
from lg_common.helpers import extract_first_asset_from_director_message
from lg_common.msg import BrowserCmdArg
from lg_common.msg import BrowserExtension


class AdhocBrowserDirectorBridge():
    """
    Bridge between director and AdhocBrowser service

    - listens on director messages
    - extracts windows/assets from director messages pointed at viewport that this bridge was configured for
    - unpacks director messages and translates/publishes them in form understandable by AdhocBrowserPool
    """

    def __init__(self,
                 aggregate_publisher,
                 viewport_browser_pool_publisher,
                 viewport_name):
        """
        AdhocBrowserDirectorBridge should be configured per each viewport to achieve nice separation and granularity
        """
        self.viewport_name = viewport_name
        self.browser_pool_publisher = viewport_browser_pool_publisher
        self.aggregate_publisher = aggregate_publisher

    def translate_director(self, data):
        """
        Translates /director/scene messages to one AdhocBrowsers message and publishes it immediately
        """

        try:
            message = json.loads(data.message)
            slug = message['slug']
        except KeyError:
            rospy.logwarn("Director message did not contain 'slug' attribute")
            return
        except AttributeError:
            rospy.logwarn("Director message did not contain valid data")
            return
        except ValueError:
            rospy.logwarn("Director message did not contain valid json")
            return

        adhoc_browsers_list, preload = self._extract_adhoc_browsers(data)

        # Add ros_window redy extension to browsers if we
        # do preloading
        if preload:
            ros_window_ready_ext = BrowserExtension()
            ros_window_ready_ext.name = 'ros_window_ready'
            for adhoc_browser in adhoc_browsers_list:
                adhoc_browser.extensions.append(ros_window_ready_ext)

        adhoc_browsers = AdhocBrowsers()
        adhoc_browsers.scene_slug = slug
        adhoc_browsers.browsers = adhoc_browsers_list

        if preload:
            adhoc_browsers.preload = True
        else:
            adhoc_browsers.preload = False

        rospy.logdebug("Publishing AdhocBrowsers: %s" % adhoc_browsers)

        self.browser_pool_publisher.publish(adhoc_browsers)
        if adhoc_browsers.browsers:
            self.aggregate_publisher.publish(adhoc_browsers)

    def _preload_scene(self, adhoc_browsers_list):
        """
        Iterates over adhoc browsers list and if ALL adhoc browsers
        on the list have preload set to true, then it's going to preload
        the scene in the background using smooth transision

        Returns bool
        """
        for browser in adhoc_browsers_list:
            if not browser.preload:
                return False
        return True

    def _get_viewport_offset(self):
        """
        Adhoc browser needs geometry that's honoring viewport offsets
        This method will add viewport offset
        """
        viewport_geometry = ManagedWindow.get_viewport_geometry()
        if not viewport_geometry:
            viewport_geometry = WindowGeometry()
        return {'x': viewport_geometry.x, 'y': viewport_geometry.y}

    def _unpack_browser_config(self, adhoc_browser, browser_config):
        """
        accepts brower_config (`activity_config` part of USCS/director message)
        and returns adhoc_browser object with detected activity_config attributes
        """
        binary = browser_config.get('binary_path', '/usr/bin/google-chrome')
        user_agent = browser_config.get('user_agent', None)
        browser_cmd_args = browser_config.get('cmd_args', None)
        extensions = browser_config.get('extensions', None)

        if binary:
            adhoc_browser.binary = binary

        if user_agent:
            adhoc_browser.user_agent = user_agent

        if browser_cmd_args:
            for cmd_arg in browser_cmd_args:
                browser_arg = BrowserCmdArg()
                browser_arg.name = cmd_arg.name
                browser_arg.path = cmd_arg.path
                browser_arg.metadata = cmd_arg.metadata
                adhoc_browser.cmd_args.append(browser_arg)

        if extensions:
            for extension in extensions:
                browser_extension = BrowserExtension()
                browser_extension.name = extension['name']
                browser_extension.path = extension['path']
                browser_extension.metadata = extension['metadata']
                adhoc_browser.extensions.append(browser_extension)

        return adhoc_browser

    def _extract_adhoc_browsers(self, data):
        """
        Returns a list containing AdhocBrowser objects extracted from director message for viewport
        specific to adhoc_browser that this instance of bridge is tied to.
        """
        rospy.logdebug("Got data on _extract_adhoc_browsers: %s" % data)
        adhoc_browsers = []
        browsers = extract_first_asset_from_director_message(data, 'browser', self.viewport_name)
        rospy.logdebug("Extracted browsers _extract_adhoc_browsers: %s" % browsers)
        preload = False

        for browser in browsers:
            # TODO (WZ) make a hash from url + geometry here
            browser_id = uuid.uuid4().hex[:8]
            browser_name = 'adhoc_browser_' + self.viewport_name + '_' + str(browser_id)
            adhoc_browser = AdhocBrowser()
            adhoc_browser.id = browser_name
            adhoc_browser.url = browser['path']
            adhoc_browser.binary = '/usr/bin/google-chrome'
            adhoc_browser.geometry.x = browser['x_coord'] + self._get_viewport_offset()['x']
            adhoc_browser.geometry.y = browser['y_coord'] + self._get_viewport_offset()['y']
            adhoc_browser.geometry.height = browser['height']
            adhoc_browser.geometry.width = browser['width']

            activity_config = browser.get('activity_config', None)

            if activity_config:
                chrome_config = activity_config.get('google_chrome', None)
                if activity_config.get('preload', None):
                    preload = True

                if chrome_config:
                    adhoc_browser = self._unpack_browser_config(adhoc_browser, chrome_config)

            adhoc_browsers.append(adhoc_browser)

        rospy.logdebug("Returning adhocbrowsers: %s" % adhoc_browsers)

        return adhoc_browsers, preload
