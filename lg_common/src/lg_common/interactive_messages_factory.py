import sys
import json
import rospy
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
                    "test_extension1"
                ]
                }
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/example/index.html"
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
        self.test_one_browser_with_allowed_urls_msg = self._create_message("""
        {
        "description": "one browser with allowed urls",
        "duration": 100,
        "name": "one browser with allowed urls",
        "resource_uri": "/director_api/scene/one_browser_with_allowed_urls/",
        "slug": "one_browser_with_allowed_urls",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "google_chrome":{
                    "allowed_urls": ["google.com", "endpoint.com"]
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
                        "test_extension1",
                        "test_extension2"
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
                    "test_extension1",
                    "test_extension2"
                ]
                }
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
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
                    "command_line_args": [
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
                    "version": "beta"
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
                "http://127.0.0.1:8008/lg_common/webapps/example/index.html"
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
                "http://127.0.0.1:8008/lg_common/webapps/example/index.html"
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
        self.test_one_browser_with_preloading_and_wrong_url_msg = self._create_message("""
        {
        "description": "one_browser_with_preloading_and_wrong_url",
        "duration": 100,
        "name": "one_browser_with_preloading_and_wrong_url",
        "resource_uri": "/director_api/scene/one_browser_with_preloading_and_wrong_url/",
        "slug": "one_browser_with_preloading_and_wrong_url",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://asdasdasdaqweqwenonexistentpage.com"
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
        self.test_one_browser_with_preloading_msg = self._create_message("""
        {
        "description": "one_browser_with_preloading",
        "duration": 100,
        "name": "one_browser_with_preloading",
        "resource_uri": "/director_api/scene/one_browser_with_preloading/",
        "slug": "one_browser_with_preloading",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
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
        self.test_one_browser_with_preloading_alt_slug_msg = self._create_message("""
        {
        "description": "one_browser_with_preloading_alt_slug",
        "duration": 100,
        "name": "one_browser_with_preloading_alt_slug",
        "resource_uri": "/director_api/scene/one_browser_with_preloading_alt_slug/",
        "slug": "one_browser_with_preloading_alt_slug",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
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
        self.test_two_browsers_with_preloading_mix_msg = self._create_message("""
        {
        "description": "one_two_browsers_with_preloading_mix",
        "duration": 100,
        "name": "two_browsers_with_preloading_mix",
        "resource_uri": "/director_api/scene/two_browsers_with_preloading_mix/",
        "slug": "two_browsers_with_preloading_mix",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
            ],
            "height": 100,
            "presentation_viewport": "center",
            "width": 100,
            "x_coord": 300,
            "y_coord": 300
            },
            {
            "activity": "browser",
            "activity_config": {
                "preload": false
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
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
        self.test_two_browsers_with_preloading_mix_alt_slug_msg = self._create_message("""
        {
        "description": "one_two_browsers_with_preloading_mix_alt_slug",
        "duration": 100,
        "name": "two_browsers_with_preloading_mix_alt_slug",
        "resource_uri": "/director_api/scene/two_browsers_with_preloading_mix_alt_slug/",
        "slug": "two_browsers_with_preloading_mix_alt_slug",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
            ],
            "height": 100,
            "presentation_viewport": "center",
            "width": 100,
            "x_coord": 300,
            "y_coord": 300
            },
            {
            "activity": "browser",
            "activity_config": {
                "preload": false
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
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
        self.test_four_browsers_with_preloading_mix_msg = self._create_message("""
        {
        "description": "four_browsers_with_preloading_mix",
        "duration": 100,
        "name": "four_browsers_with_preloading_mix",
        "resource_uri": "/director_api/scene/four_browsers_with_preloading_mix/",
        "slug": "four_browsers_with_preloading_mix",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
            ],
            "height": 100,
            "presentation_viewport": "center",
            "width": 100,
            "x_coord": 300,
            "y_coord": 300
            },
            {
            "activity": "browser",
            "activity_config": {
                "preload": false
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
            ],
            "height": 333,
            "presentation_viewport": "center",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            },
            {
            "activity": "browser",
            "activity_config": {
                "preload": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
            ],
            "height": 100,
            "presentation_viewport": "left",
            "width": 100,
            "x_coord": 300,
            "y_coord": 300
            },
            {
            "activity": "browser",
            "activity_config": {
                "preload": false
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/test.html"
            ],
            "height": 333,
            "presentation_viewport": "left",
            "width": 333,
            "x_coord": 22,
            "y_coord": 22
            }
        ]
        }
        """)
        self.test_one_browser_with_preloading_and_custom_preloading_event_msg = self._create_message("""
        {
        "description": "one_browser_with_preloading_and_custom_preloading_event",
        "duration": 100,
        "name": "one_browser_with_preloading_and_custom_preloading_event",
        "resource_uri": "/director_api/scene/one_browser_with_preloading_and_custom_preloading_event/",
        "slug": "one_browser_with_preloading_and_custom_preloading_event",
        "windows": [
            {
            "activity": "browser",
            "activity_config": {
                "preload": true,
                "custom_preload_event": true
            },
            "assets": [
                "http://127.0.0.1:8008/lg_common/webapps/window_ready_mock/custom_event.html?use_app_event=1"
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

    def emit_message(self, ivar_name):
        """
        emits message using instance variable name
        """
        if not self.publisher:
            self.publisher = rospy.Publisher(
                '/director/scene', GenericMessage, queue_size=3
            )
            rospy.init_node("ispaces_messages_factory")
        message = self._get_message(ivar_name)
        print("message string: %s" % message)
        self.publisher.publish(message)
        return True

    def _create_message(self, msg_string):
        message = GenericMessage()
        message.type = 'json'
        try:
            message_json = json.loads(msg_string)
            message.message = json.dumps(message_json)
            return message
        except ValueError:
            print("Could not decode json message from InteractiveSpacesMessagesFactory")
            sys.exit(1)

    def _init_publisher(self):
        pass

    def _get_message(self, message_name):
        """
        Returns message as json
        """
        return getattr(self, message_name)


if __name__ == "__main__":
    try:
        messages = InteractiveSpacesMessagesFactory()
        message_name = sys.argv[1]
        print("Emitting %s message" % message_name)
        messages.emit_message(message_name)
    except IndexError:
        print("")
        print("This file, if called directly, will emit an interactivespaces.msgs.GenericMessage")
        print("")
        print("You must provide message name to emit:\n%s" %
              '\n'.join(["- " + ivar for ivar in dir(messages) if ivar.startswith('test_')]))
        print("")
        print("NOTE: methods beginning with 'test' are used by test suite")
        print("")
        sys.exit(1)
