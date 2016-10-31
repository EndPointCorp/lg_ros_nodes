import time

from interactivespaces_msgs.msg import GenericMessage


TEST_MESSAGE_TEMPLATE = """
{{
  "name": "foobar_name",
  "description": "test_description",
  "resource_uri": "test_foobar_uri",
  "slug": "test_foobar_slug",
  "duration": 0,
  "windows": [
    {windows}
  ]
}}
"""

TOUCH_WINDOW_TEMPLATE = """
{{
  "activity": "{activity}",
  "activity_config": {{
    "route_touch": {route_touch},
    "viewport": "viewport://{source}"
  }},
  "assets": [
    "viewport://{source}"
  ],
  "width": 640,
  "height": 480,
  "x_coord": 0,
  "y_coord": 0,
  "presentation_viewport": "{target}"
}}
"""

BROWSER_WINDOW_TEMPLATE = """
{{
  "activity": "browser",
  "activity_config": {{
    "route_touch": {route_touch}
  }},
  "assets": [
    "https://cnn.com"
  ],
  "width": 640,
  "height": 480,
  "x_coord": 0,
  "y_coord": 0,
  "presentation_viewport": "{target}"
}}
"""


def gen_touch_window(
        route,
        source,
        target='default_viewport',
        activity='foobar_activity'):
    route_touch = 'true' if route else 'false'
    return TOUCH_WINDOW_TEMPLATE.format(
        activity=activity,
        source=source,
        target=target,
        route_touch=route_touch,
    )


def gen_browser_window(
        route,
        target='default_viewport'):
    route_touch = 'true' if route else 'false'
    return BROWSER_WINDOW_TEMPLATE.format(
        target=target,
        route_touch=route_touch
    )


def gen_scene(windows):
    joined_windows = ', '.join(windows)
    return TEST_MESSAGE_TEMPLATE.format(windows=joined_windows)


def gen_scene_msg(scene):
    return GenericMessage(type='json', message=scene)


def wait_for_assert_equal(val1, val2, timeout):
    """
    Waits for two values to become equal within specified timeout
    """
    for iteration in xrange(0, timeout):
        if val1 == val2:
            return True
        else:
            time.sleep(1)

    return False
