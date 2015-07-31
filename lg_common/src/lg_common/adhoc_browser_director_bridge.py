import rospy

from lg_common.msg import AdhocBrowser
from lg_common.msg import AdhocBrowsers
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import extract_first_asset_from_director_message


class AdhocBrowserDirectorBridge():
    """
    Bridge between director and AdhocBrowser service

    - listens on director messages
    - extracts windows/assets from director messages pointed at viewport that this bridge was configured for
    - unpacks director messages and translates/publishes them in form understandable by AdhocBrowserPool
    """

    def __init__(self, browser_pool_publisher, viewport_name):
        self.viewport_name = viewport_name
        self.browser_pool_publisher = browser_pool_publisher

    def translate_director(self, data):
        """
        Translates /director/scene messages to one AdhocBrowsers message
        """
        adhoc_browsers_list = self._extract_adhoc_browsers(data)

        adhoc_browsers = AdhocBrowsers()
        adhoc_browsers.browsers = adhoc_browsers_list
        self.browser_pool_publisher.publish(adhoc_browsers)

    def _extract_adhoc_browsers(self, data):
        """
        Returns a list containing AdhocBrowser objects that should be sent to
        viewport specific to this bridge
        """
        adhoc_browsers = []
        browser_id = 0
        browsers = extract_first_asset_from_director_message(data, 'browser', self.viewport_name)
        for browser in browsers:
            browser_name = 'adhoc_browser_' + self.viewport_name + '_' + browser_id
            geometry = ""
            adhoc_browser = AdhocBrowser(geometry=geometry,
                                         url=browser['path'],
                                         id=browser_name)
            browser_id += 1

        return adhoc_browsers
