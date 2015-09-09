#!/usr/bin/env python

PKG = 'lg_attract_loop'
NAME = 'test_lg_attract_loop'

import rospy
import unittest
import json

from lg_attract_loop import AttractLoop
from std_msgs.msg import Bool


class MockAPI:
    def __init__(self):
        self.presentation_group = {
                                "meta": {
                                    "limit": 1000,
                                    "next": None,
                                    "offset": 0,
                                    "previous": None,
                                    "total_count": 1
                                },
                                "objects": [
                                    {
                                    "attract_loop": True,
                                    "description": "Test",
                                    "name": "Test",
                                    "resource_uri": "/director_api/presentationgroup/kml-test/",
                                    "slug": "kml-test"
                                    }
                                ]
                                }

        self.presentations = {
                            "attract_loop": True,
                            "description": "Test",
                            "name": "Test",
                            "presentations": [
                                {
                                "description": "test presentation",
                                "name": "Test presentation",
                                "resource_uri": "/director_api/presentation/kml-test-presentation/",
                                "slug": "kml-test-presentation"
                                }
                            ],
                            "resource_uri": "/director_api/presentationgroup/kml-test/",
                            "slug": "kml-test"
                            }

        self.presentation = \
            {"description": "test presentation",
            "name": "Test presentation",
            "resource_uri": "/director_api/presentation/kml-test-presentation/",
            "scenes": [
                {
                "description": "Openflights data",
                "duration": 1,
                "name": "Flights",
                "resource_uri": "/director_api/scene/flights/",
                "slug": "flights"
                },
                {
                "description": "",
                "duration": 1,
                "name": "Mplayer two different instances",
                "resource_uri": "/director_api/scene/mplayer-two-different-instances/",
                "slug": "mplayer-two-different-instances"
                }
            ],
            "slug": "kml-test-presentation"
            }

        self.mplay_scene = {
            "description": "",
            "duration": 1,
            "name": "Mplayer two different instances",
            "resource_uri": "/director_api/scene/mplayer-two-different-instances/",
            "slug": "mplayer-two-different-instances",
            "windows": [
                {
                "activity": "video",
                "assets": [
                    "http://lg-head/lg/assets/videos/1222.mp4"
                ],
                "height": 500,
                "presentation_viewport": "left_one",
                "width": 500,
                "x_coord": 200,
                "y_coord": 200
                }
            ]
            }

        self.flights_scene = {
            "description": "Openflights data",
            "duration": 1000,
            "name": "Flights",
            "resource_uri": "/director_api/scene/flights/",
            "slug": "flights",
            "windows": [
                {
                "activity": "earth",
                "assets": [
                    "http://www.doogal.co.uk/LondonTubeLinesKML.php",
                    "http://openflights.org/demo/openflights-sample.kml"
                ],
                "height": 1,
                "presentation_viewport": "center",
                "width": 1,
                "x_coord": 1,
                "y_coord": 1
                }
            ]
            }

    def get(self, url):
        rospy.loginfo("MockAPI call %s" % url)
        if url == '/director_api/presentationgroup/?attract_loop=True':
            return json.dumps(self.presentation_group)
        elif url == '/director_api/presentationgroup/kml-test/':
            return json.dumps(self.presentations)
        elif url == '/director_api/presentation/kml-test-presentation/':
            return json.dumps(self.presentation)
        elif url == '/director_api/scene/flights/?format=json':
            return json.dumps(self.flights_scene)
        elif url == '/director_api/scene/mplayer-two-different-instances/?format=json':
            return json.dumps(self.mplayer_scene)
        else:
            rospy.logerr("Unknown call to MockAPI %s" % url)


class MockDirectorScenePublisher:
    def __init__(self):
        self.published_scenes = []

    def publish(self, message):
        rospy.loginfo("MDSP publishing message %s" % message)
        self.published_scenes.append(message)


class MockDirectorPresentationPublisher:
    def __init__(self):
        self.published_presentations = []

    def publish(self, message):
        rospy.loginfo("MDPP publishing message %s" % message)
        self.published_presentations.append(message)


class MockEarthQueryPublisher:
    def __init__(self):
        self.published_messages = []

    def publish(self, message):
        rospy.loginfo("MEQP publishing message %s" % message)
        self.published_messages.append(message)


class TestAttractLoop(unittest.TestCase):
    def setUp(self):
        rospy.init_node("lg_attract_loop_testing")
        self.mock_api = MockAPI()
        self.mock_director_scene_publisher = MockDirectorScenePublisher()
        self.mock_director_presentation_publisher = MockDirectorPresentationPublisher()
        self.earth_query_publisher = MockEarthQueryPublisher()
        self.stop_action = 'go_blank'
        self.attract_loop_controller = AttractLoop(api_proxy=self.mock_api,
                                              director_scene_publisher=self.mock_director_scene_publisher,
                                              director_presentation_publisher=self.mock_director_presentation_publisher,
                                              stop_action=self.stop_action,
                                              earth_query_publisher=self.earth_query_publisher,
                                              default_presentation=None)

    def test_basic(self):

        self.assertEqual(1,1)
        self.assertEqual(isinstance(self.attract_loop_controller, AttractLoop), True)
        self.assertEqual(self.attract_loop_controller.attract_loop_queue, [])

        # game is on
        self.attract_loop_controller.play_loop = True
        rospy.loginfo("Sleeping 2 seconds before making asserts")
        rospy.sleep(3)

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 1)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAttractLoop)
