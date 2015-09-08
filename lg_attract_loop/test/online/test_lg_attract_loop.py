#!/usr/bin/env python

PKG = 'lg_attract_loop'
NAME = 'test_lg_attract_loop'

import rospy
import unittest
import json

from lg_attract_loop import AttractLoop
from random import randint


class MockAPI:
    def __init__(self):
        self.url = randint(61530, 65530)

    def get_url(self):
        return self.url


class MockDirectorScenePublisher:
    def __init__(self):
        self.published_scenes = []

    def publish(self, message):
        self.published_messages.append(message)


class MockDirectorPresentationPublisher:
    def __init__(self):
        self.published_presentations = []

    def publish(self, message):
        self.published_presentations.append(message)


class MockEarthQueryPublisher:
    def __init__(self):
        self.published_messages = []

    def publish(self, message):
        self.published_messages.append(message)


class TestAttractLoop(unittest.TestCase):
    def _init_mocks(self):
        self.mock_api = MockAPI()
        self.api_url = self.mock_api.get_url()
        self.mock_director_scene_publisher = MockDirectorScenePublisher()
        self.mock_director_presentation_publisher = MockDirectorPresentationPublisher()
        self.earth_query_publisher = MockEarthQueryPublisher()

    def test_basic(self):
        self._init_mocks()
        stop_action = 'go_blank'
        attract_loop_controller = AttractLoop(api_url = MockAPI().get_url(),
                                              director_scene_publisher=self.mock_director_scene_publisher,
                                              director_presentation_publisher=self.mock_director_presentation_publisher,
                                              stop_action=stop_action,
                                              earth_query_publisher=self.earth_query_publisher,
                                              default_presentation=None)

        self.assertEqual(1,1)
        self.assertEqual(isinstance(attract_loop_controller, AttractLoop), True)
        self.assertEqual(attract_loop_controller.attract_loop_queue, [])


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAttractLoop)
