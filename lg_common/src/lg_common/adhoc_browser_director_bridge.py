import rospy
import json

from lg_common import ManagedWindow
from lg_msg_defs.msg import AdhocBrowser
from lg_msg_defs.msg import AdhocBrowsers
from lg_msg_defs.msg import BrowserCmdArg
from lg_msg_defs.msg import WindowGeometry
from lg_msg_defs.msg import BrowserExtension
from lg_common.helpers import generate_hash
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import extract_first_asset_from_director_message
from lg_common.logger import get_logger
logger = get_logger('adhoc_browser_director_bridge')


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

        Publishes AdhocBrowsers message on a viewport browser service topic
        Publishes AdhocBrowsers message on a common topic only if there are some browsers
        in the message
        Takes care of `preload` flag and adds it to ManagedAdhocBrowser instance if
        browser should be preloaded. It emits a message with ros_window_ready extension
        if above constraints were met.
        """

        try:
            message = json.loads(data.message)
            slug = message['slug']
        except KeyError:
            logger.error("Director message did not contain 'slug' attribute")
            return
        except AttributeError:
            logger.error("Director message did not contain valid data")
            return
        except ValueError:
            logger.error("Director message did not contain valid json")
            return
        except TypeError:
            logger.error("Director message did not contain valid type. Type was %s, and content was: %s" % (type(message), message))
            return

        adhoc_browsers_list = self._extract_browsers_from_message(data)

        adhoc_browsers = AdhocBrowsers()
        adhoc_browsers.scene_slug = slug
        adhoc_browsers.browsers = adhoc_browsers_list

        logger.debug("Publishing AdhocBrowsers: %s" % adhoc_browsers)

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
        version = browser_config.get('version', 'stable')
        user_agent = browser_config.get('user_agent', None)
        browser_cmd_args = browser_config.get('command_line_args', None)
        default_args_removal = browser_config.get('default_args_removal', None)
        extensions = browser_config.get('extensions', None)
        allowed_urls = browser_config.get('allowed_urls', None)
        kiosk = browser_config.get('kiosk', True)
        user_data_dir = browser_config.get('user_data_dir', None)

        adhoc_browser.kiosk = kiosk

        if version:
            adhoc_browser.version = version

        if user_agent:
            adhoc_browser.user_agent = user_agent

        if browser_cmd_args:
            for cmd_arg in browser_cmd_args:
                browser_arg = BrowserCmdArg()
                browser_arg.argument = str(cmd_arg)
                adhoc_browser.command_line_args.append(browser_arg)

        if default_args_removal:
            logger.debug("there were args to remove")
            for cmd_arg in default_args_removal:
                logger.debug("One arg was..")
                browser_arg = BrowserCmdArg()
                browser_arg.argument = str(cmd_arg)
                adhoc_browser.default_args_removal.append(browser_arg)

        if extensions:
            for extension in extensions:
                browser_extension = BrowserExtension()
                if isinstance(extension, str):
                    browser_extension.name = str(extension)
                else:
                    browser_extension.name = str(extension['name'])
                adhoc_browser.extensions.append(browser_extension)

        if allowed_urls:
            for aurl in allowed_urls:
                adhoc_browser.allowed_urls.append(str(aurl))

        if user_data_dir:
            adhoc_browser.user_data_dir = user_data_dir

        return adhoc_browser

    def _extract_browsers_from_message(self, data):
        """
        Returns a list containing AdhocBrowser objects extracted from director message for viewport
        specific to adhoc_browser that this instance of bridge is tied to.

        Each browser has a unique hash assigned to it. It's generated on the basis
        of browser's attributes.
        """
        logger.debug("Got data on _extract_browsers_from_message: %s" % data)
        adhoc_browsers = []
        browsers = extract_first_asset_from_director_message(data, 'browser', self.viewport_name)
        logger.debug("Extracted browsers _extract_browsers_from_message: %s" % browsers)
        message = json.loads(data.message)

        for browser in browsers:
            adhoc_browser = AdhocBrowser()
            adhoc_browser.scene_slug = message['slug']
            adhoc_browser.url = browser['path']
            adhoc_browser.version = 'stable'
            adhoc_browser.geometry.x = browser['x_coord'] + self._get_viewport_offset()['x']
            adhoc_browser.geometry.y = browser['y_coord'] + self._get_viewport_offset()['y']
            adhoc_browser.geometry.height = browser['height']
            adhoc_browser.geometry.width = browser['width']
            adhoc_browser.preload = False  # it's a default value
            adhoc_browser.kiosk = True  # also default

            activity_config = browser.get('activity_config', None)

            if activity_config:
                if activity_config.get('preload', None):
                    adhoc_browser.preload = True

                custom_preload_event = activity_config.get('custom_preload_event', None)

                if custom_preload_event is True:
                    adhoc_browser.custom_preload_event = True
                else:
                    adhoc_browser.custom_preload_event = False

                chrome_config = activity_config.copy()
                chrome_config.update(activity_config.get('google_chrome', {}))
                adhoc_browser = self._unpack_browser_config(adhoc_browser, chrome_config)

            if adhoc_browser.preload:
                browser_id = generate_hash(self._serialize_adhoc_browser(adhoc_browser), random_suffix=True)
            else:
                browser_id = generate_hash(self._serialize_adhoc_browser(adhoc_browser))

            adhoc_browser.id = browser_id

            adhoc_browsers.append(adhoc_browser)

        logger.debug("Returning adhocbrowsers: %s" % adhoc_browsers)

        return adhoc_browsers

    def _serialize_adhoc_browser(
            self,
            adhoc_browser,
            filtered_slots=[
                'scene_slug',
                'allowed_urls'
            ]):
        """
        Serialize adhoc browser but filter out
        scene slug that it belongs to, to prevent
        adhoc_browser hash from being unique per scene
        """
        serialized_browser_string = ''

        for slot in adhoc_browser.__slots__:
            if slot not in filtered_slots:
                serialized_browser_string += getattr(adhoc_browser, slot).__str__()

        return serialized_browser_string
