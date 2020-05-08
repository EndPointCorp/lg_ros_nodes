#!/usr/bin/env python3

"""
test_onboard_router - unit tests for onboard_router.py
offline tests - not requiring the ROS node itself to run.

"""


import os

import pytest
import rospkg
from std_msgs.msg import Bool

from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.msg import StringArray
from lg_keyboard import OnboardRouter


DIRECTOR_MESSAGE_NO_WINDOWS = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message"
    }"""

DIRECTOR_MESSAGE = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "mirror",
                "height": 1080,
                "presentation_viewport": "center",
                "assets": [
                    ],
                "width": 450,
                "height": 800,
                "x_coord": 0,
                "y_coord": 0,

                "activity_config": {
                    "viewport": "center",
                    "route_touch": true
                }
            }
            ]
    }"""


class MockActivatePublisher(object):

    def __init__(self):
        self.msgs = []

    def publish(self, msgs):
        self.msgs.append(msgs)


class TestOnboardRouter(object):

    def setUp(self):
        self.publisher = MockActivatePublisher()
        self.msg = GenericMessage()
        self.msg.type = "json"
        self.router = OnboardRouter(default_viewport=["wall"],
                                    onboard_activate_publisher=self.publisher)

    def test_handle_scene_no_windows(self):
        """
        Upon sending a director message with no windows defined,
        the router publishes empty StringArray

        """
        self.msg.message = DIRECTOR_MESSAGE_NO_WINDOWS
        self.router.handle_scene(self.msg)
        assert self.publisher.msgs[0].strings == []

    def test_hand_scene_with_windows(self):
        """
        Upon sending a director message with windows,
        the onboard shall first be shut.
        Assert on correct viewports settings.

        """
        self.msg.message = DIRECTOR_MESSAGE
        assert self.router.default_viewport == self.router.active_viewport == ["wall"]
        self.router.handle_scene(self.msg)
        assert self.publisher.msgs[0].strings == []
        assert self.router.active_viewport == ["center"]

    def test_handle_visibility_hide_onboard(self):
        msg = Bool(data=False)
        assert self.router.last_state is None
        self.router.handle_visibility(msg)
        assert self.router.last_state is False
        assert self.publisher.msgs[0].strings == []
        # check that the hide message was sent just once
        assert len(self.publisher.msgs) == 1
        # check that repeatedly sending messages of the same value
        # won't get processed multiple times but only once
        for _ in range(10):
            self.router.handle_visibility(msg)
        assert self.router.last_state is False
        assert self.publisher.msgs[0].strings == []
        # check that the hide message was sent just once
        assert len(self.publisher.msgs) == 1

    def test_handle_visibility_show_onboard(self):
        msg = Bool(data=True)
        assert self.router.last_state is None
        self.router.handle_visibility(msg)
        assert self.router.last_state is True
        assert self.publisher.msgs[0].strings == ["wall"]
        # check that the hide message was sent just once
        assert len(self.publisher.msgs) == 1
        # check that repeatedly sending messages of the same value
        # won't get processed multiple times but only once
        for _ in range(10):
            self.router.handle_visibility(msg)
        assert self.router.last_state is True
        assert self.publisher.msgs[0].strings == ["wall"]
        # check that the hide message was sent just once
        assert len(self.publisher.msgs) == 1


if __name__ == "__main__":
    test_pkg = "lg_keyboard"
    test_name = "test_onboard_router"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
