"""
Test module for all helper functions from the helpers.py module

"""


import os

import pytest
import rospy
import rospkg
import rostopic
import unittest

from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import extract_first_asset_from_director_message
from lg_common.helpers import load_director_message
from lg_common.helpers import unpack_activity_sources


DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOT_PRESENT = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "video",
                "assets": [
                    "whatever"
                ],
                "height": 1080,
                "presentation_viewport": "center",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0
            }
            ]
    }"""

DIRECTOR_MESSAGE_ACTIVITY_CONFIG_EMPTY = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "video",
                "assets": [
                    "whatever"
                ],
                "height": 1080,
                "presentation_viewport": "center",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0,
                "activity_config": {}
            }
            ]
    }"""

DIRECTOR_MESSAGE_ACTIVITY_CONFIG_LOOP = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "video",
                "assets": [
                    "whatever"
                ],
                "height": 1080,
                "presentation_viewport": "center",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0,
                "activity_config": {
                    "onFinish": "loop"
                }
            }
            ]
    }"""

DIRECTOR_MESSAGE_ACTIVITY_CONFIG_CLOSE = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "video",
                "assets": [
                    "whatever"
                ],
                "height": 1080,
                "presentation_viewport": "center",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0,
                "activity_config": {
                    "onFinish": "close"
                }
            }
            ]
    }"""

DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOTHING = """
    {
            "description": "bogus",
            "duration": 0,
            "name": "test whatever",
            "resource_uri": "bogus",
            "slug": "test message",
            "windows": [
            {
                "activity": "video",
                "assets": [
                    "whatever"
                ],
                "height": 1080,
                "presentation_viewport": "center",
                "width": 1920,
                "x_coord": 0,
                "y_coord": 0,
                "activity_config": {
                    "onFinish": "nothing"
                }
            }
            ]
    }"""


class TestHelpers(unittest.TestCase):
    def setUp(self):
        self.msg = GenericMessage()
        self.msg.type = "json"

    def test_load_director_message_wrong_json(self):
        self.msg.message = "wrong json"
        with self.assertRaises(ValueError):
            load_director_message(self.msg)

    def test_load_director(self):
        self.msg.message = DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOTHING
        d = load_director_message(self.msg)
        self.assertIsInstance(d, dict)
        self.assertEqual(d["windows"][0]["activity_config"]["onFinish"], "nothing")

    def test_extract_first_asset_from_director_message_return_empty_list(self):
        self.msg.message = DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOTHING
        r = extract_first_asset_from_director_message(self.msg, "something", "center")
        # get empty list since activity type does not match
        assert r == []
        r = extract_first_asset_from_director_message(self.msg, "video", "somewhereelse")
        # get empty list since viewport does not match
        assert r == []
        r = extract_first_asset_from_director_message(self.msg, "something", "somewhereelse")
        assert r == []

    def test_extract_first_asset_from_director_message_general(self):
        self.msg.message = DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOTHING
        #extract_first_asset_from_director_message(message, activity_type, viewport)
        r = extract_first_asset_from_director_message(self.msg, "video", "center")
        assert r[0]["x_coord"] == 0
        assert r[0]["y_coord"] == 0
        assert r[0]["height"] == 1080
        assert r[0]["width"] == 1920

    def test_extract_first_asset_from_director_message_activity_config_options(self):
        # no activity_config attribute present
        self.msg.message = DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOT_PRESENT
        r = extract_first_asset_from_director_message(self.msg, "video", "center")
        assert not hasattr(r[0], "on_finish")
        # activity_config present but empty
        self.msg.message = DIRECTOR_MESSAGE_ACTIVITY_CONFIG_EMPTY
        r = extract_first_asset_from_director_message(self.msg, "video", "center")
        assert not hasattr(r[0], "on_finish")
        # activity_config present, onFinish close, loop, nothing
        for m, on_finish in ((DIRECTOR_MESSAGE_ACTIVITY_CONFIG_LOOP, "loop"),
                             (DIRECTOR_MESSAGE_ACTIVITY_CONFIG_CLOSE, "close"),
                             (DIRECTOR_MESSAGE_ACTIVITY_CONFIG_NOTHING, "nothing")):
            self.msg.message = m
            r = extract_first_asset_from_director_message(self.msg, "video", "center")
            assert r[0]["on_finish"] == on_finish

    def test_unpack_activity_sources(self):
        source_string = "/touchscreen/touch:interactivespaces_msgs/GenericMessage:activity"
        result = [{"topic": "/touchscreen/touch",
                   "message_type": "interactivespaces_msgs/GenericMessage",
                   "strategy": "activity",
                   "slot": None,
                   "value_min": None,
                   "value_max": None,
                   "value": None}]
        assert result == unpack_activity_sources(source_string)

        source_string = "/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5"
        result = [{"topic": "/proximity_sensor/distance",
                   "message_type": "sensor_msgs/Range",
                   "strategy": "value",
                   "slot": "range",
                   "value_min": "0",
                   "value_max": "2.5",
                   "value": None}]
        assert result == unpack_activity_sources(source_string)

        source_string = "/proximity_sensor/distance:sensor_msgs/Range-range:average"
        result = [{"topic": "/proximity_sensor/distance",
                   "message_type": "sensor_msgs/Range",
                   "strategy": "average",
                   "slot": "range",
                   "value_min": None,
                   "value_max": None,
                   "value": None}]
        assert result == unpack_activity_sources(source_string)

        source_string = ("/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5;"
                         "/touchscreen/touch:interactivespaces_msgs/GenericMessage:delta")
        result = [{"topic": "/proximity_sensor/distance",
                   "message_type": "sensor_msgs/Range",
                   "strategy": "value",
                   "slot": "range",
                   "value_min": "0",
                   "value_max": "2.5",
                   "value": None},
                  {"topic": "/touchscreen/touch",
                   "message_type": "interactivespaces_msgs/GenericMessage",
                   "slot": None,
                   "strategy": "delta",
                   "value_min": None,
                   "value_max": None,
                   "value": None}]
        assert result == unpack_activity_sources(source_string)

        source_string = ("/earth/query/search:std_msgs/String-data:default;"
                         "/lg_replay/touchscreen:interactivespaces_msgs/GenericMessage-message:count;"
                         "/spacenav/twist:geometry_msgs/Twist-angular:count_nonzero;"
                         "/proximity/distance:sensor_msgs/Range-range:average")
        result = [{"topic": "/earth/query/search",
                   "message_type": "std_msgs/String",
                   "strategy": "default",
                   "slot": "data",
                   "value_min": None,
                   "value_max": None,
                   "value": None},
                  {"topic": "/lg_replay/touchscreen",
                   "message_type": "interactivespaces_msgs/GenericMessage",
                   "slot": "message",
                   "strategy": "count",
                   "value_min": None,
                   "value_max": None,
                   "value": None},
                  {"topic": "/spacenav/twist",
                   "message_type": "geometry_msgs/Twist",
                   "slot": "angular",
                   "strategy": "count_nonzero",
                   "value_min": None,
                   "value_max": None,
                   "value": None},
                  {"topic": "/proximity/distance",
                   "message_type": "sensor_msgs/Range",
                   "slot": "range",
                   "strategy": "average",
                   "value_min": None,
                   "value_max": None,
                   "value": None}]
        assert result == unpack_activity_sources(source_string)

        source_string = ("/appctl/mode:appctl/Mode-mode:value-tactile;"
                         "/director/scene:interactivespaces_msgs/GenericMessage-message.slug:value-online_scene")
        result = [{"topic": "/appctl/mode",
                   "message_type": "appctl/Mode",
                   "slot": "mode",
                   "strategy": "value",
                   "value_min": None,
                   "value_max": None,
                   "value": "tactile"},
                  {"topic": "/director/scene",
                   "message_type": "interactivespaces_msgs/GenericMessage",
                   "slot": "message.slug",
                   "strategy": "value",
                   "value_min": None,
                   "value_max": None,
                   "value": "online_scene"}]
        assert result == unpack_activity_sources(source_string)
