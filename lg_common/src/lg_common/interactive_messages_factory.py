import sys
import json
from interactivespaces_msgs.msg import GenericMessage


class InteractiveSpacesMessagesFactory:
    """
    This class contains easy access to test messages
    """
    def __init__(self):
        self.publisher = None
        self.test_one_browser_with_extension_msg = self._create_message("""
        {
        "description": "one browser with extension",
        "duration": 100,
        "name": "one browser with extension",
        "resource_uri": "/director_api/scene/one_browser_with_extension/",
        "slug": "one_browser_with_extension",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "google_chrome":{
                "extensions": [
                    {
                    "name": "test_extension1"
                    }
                ]
                }
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_with_two_extensions_msg = self._create_message("""
        {
        "description": "one browser with extension",
        "duration": 100,
        "name": "one browser with extension",
        "resource_uri": "/director_api/scene/one_browser_with_extension/",
        "slug": "one_browser_with_extension",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "google_chrome":{
                "extensions": [
                    {
                    "name": "test_extension1"
                    },
                    {
                    "name": "test_extension2"
                    }
                ]
                }
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_with_two_extensions_and_preloading_msg = self._create_message("""
        {
        "description": "one_browser_tiwh_two_extensions_and_preloading",
        "duration": 100,
        "name": "one_browser_tiwh_two_extensions_and_preloading",
        "resource_uri": "/director_api/scene/one_browser_tiwh_two_extensions_and_preloading/",
        "slug": "one_browser_tiwh_two_extensions_and_preloading",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true,
                "google_chrome":{
                "extensions": [
                    {
                    "name": "test_extension1"
                    },
                    {
                    "name": "test_extension2"
                    }
                ]
                }
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_with_custom_cmdargs_msg = self._create_message("""
        {
        "description": "one browser with extension",
        "duration": 100,
        "name": "one browser with extension",
        "resource_uri": "/director_api/scene/one_browser_with_extension/",
        "slug": "one_browser_with_extension",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "google_chrome":{
                    "additional_cmd_args": [
                        "--disable-out-of-process-pac",
                        "--enable-benchmarking",
                        "--enable-crash-reporter"
                    ]
                }
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_with_custom_user_agent_msg = self._create_message("""
        {
        "description": "custom user agent",
        "duration": 100,
        "name": "custom_user_agent",
        "resource_uri": "/director_api/scene/one_browser_with_custom_user_agent/",
        "slug": "one_browser_with_custom_user_agent",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "google_chrome":{
                    "user_agent": "loltestlmfaorofl"
                }
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_with_custom_binary_msg = self._create_message("""
        {
        "description": "custom binary",
        "duration": 100,
        "name": "custom_user_agent",
        "resource_uri": "/director_api/scene/one_browser_with_custom_binary/",
        "slug": "one_browser_with_custom_binary",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "google_chrome":{
                    "binary_path": "/tmp/custom-chrome-binary"
                }
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_on_center_alt_slug_msg = self._create_message("""
        {
        "description": "one_browser_on_center_alt_slug",
        "duration": 100,
        "name": "one_browser_on_center_alt_slug",
        "resource_uri": "/director_api/scene/one_browser_on_center_alt_slug/",
        "slug": "one_browser_on_center_alt_slug",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_on_center_msg = self._create_message("""
        {
        "description": "one_browser_on_center",
        "duration": 100,
        "name": "custom_user_agent",
        "resource_uri": "/director_api/scene/one_browser_on_center/",
        "slug": "one_browser_on_center",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
            },
            "assets": [
                "https://maps.google.com"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_no_browsers_msg = self._create_message("""
        {
        "description": "no_browsers",
        "duration": 100,
        "name": "no_browsers",
        "resource_uri": "/director_api/scene/no_browsers/",
        "slug": "no_browsers",
        "windows": [
        ]
        }
        """)

    def _create_message(self, msg_string):
        message = GenericMessage()
        message.type = 'json'
        try:
            message_json = json.loads(msg_string)
            message.message = json.dumps(message_json)
            return message
        except ValueError:
            print "Could not decode json message from InteractiveSpacesMessagesFactory"
            sys.exit(1)

    def _init_publisher(self):
        pass

    def test_one_browser_with_extension(self):
        print 'one_browser_with_extension'

    def test_one_browser_with_two_extensions(self):
        print 'one_browser_with_2_extensions'

    def test_one_browser_with_two_extensions_and_preloading(self):
        print 'one_browser_with_2_extensions'

    def test_two_browsers_with_extension(self):
        print 'two_browser_with_ext'

    def _get_message(self, message_name):
        """
        Returns message as json
        """
        return getattr(self, message_name + "_msg")


if __name__ == "__main__":
    try:
        messages = InteractiveSpacesMessagesFactory()
        message_name = sys.argv[1]
        print "Emitting %s message" % message_name
        getattr(messages, message_name)()
    except IndexError:
        print ""
        print "This file, if called directly, will emit an interactivespaces.msgs.GenericMessage"
        print ""
        print "You must provide message name to emit:\n%s" % \
            '\n'.join(["- " + method for method in dir(messages) if callable(getattr(messages, method)) and not method.startswith('_')])
        print ""
        print "NOTE: methods beginning with 'test' are used by test suite"
        print ""
        sys.exit(1)
