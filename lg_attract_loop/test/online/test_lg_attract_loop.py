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

        self.mplayer_scene = {
            "description": "",
            "duration": 10000,
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
        self.maxDiff = None
        rospy.init_node("lg_attract_loop_testing")

    def _init_mocks(self):
        self.mock_api = MockAPI()
        self.mock_director_scene_publisher = MockDirectorScenePublisher()
        self.mock_director_presentation_publisher = MockDirectorPresentationPublisher()
        self.earth_query_publisher = MockEarthQueryPublisher()

    def _deactivate_lg(self):
        rospy.loginfo("Changing LG state to inactive to start playback")
        self.attract_loop_controller._process_activity_state_change(Bool(data=False))
        rospy.loginfo("Sleeping 2 seconds before making asserts")
        rospy.sleep(2)

    def _activate_lg(self):
        rospy.loginfo("Activating LG - attract looop should stop")
        self.attract_loop_controller._process_activity_state_change(Bool(data=True))
        rospy.loginfo("Sleeping 2 seconds for the attract loop to stop")
        rospy.sleep(2)

    def test_1_entering_and_exiting_attract_loop_with_go_blank(self):
        self._init_mocks()
        self.stop_action = 'go_blank'
        self.attract_loop_controller = AttractLoop(api_proxy=self.mock_api,
                                              director_scene_publisher=self.mock_director_scene_publisher,
                                              director_presentation_publisher=self.mock_director_presentation_publisher,
                                              stop_action=self.stop_action,
                                              earth_query_publisher=self.earth_query_publisher,
                                              default_presentation=None)

        self.assertEqual(isinstance(self.attract_loop_controller, AttractLoop), True)
        self.assertEqual(self.attract_loop_controller.play_loop, False)
        self.assertEqual(self.attract_loop_controller.attract_loop_queue, [])

        rospy.loginfo("game is on - first wait for initialization and check whether attract loop was populated with scenes")
        rospy.sleep(2)
        self.assertEqual(len(self.attract_loop_controller.attract_loop_queue), 2) # two scenes waiting for playback
        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 0) # no scenes published yet

        self._deactivate_lg()

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 1) # one scene playing back right now (1000 seconds :))
        self.assertEqual(len(self.attract_loop_controller.attract_loop_queue), 1)  # the other one in the queue
        self.assertEqual(self.attract_loop_controller.attract_loop_queue[0]['scene']['slug'], self.mock_api.mplayer_scene['slug']) # mplayer scene is waiting for publication
        self.assertEqual(self.attract_loop_controller.attract_loop_queue[0]['scene']['description'], self.mock_api.mplayer_scene['description']) # mplayer scene again
        self.assertEqual(self.attract_loop_controller.scene_timer > 500, True) # scene timer should be sth lik 997 here
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[0].message), self.mock_api.flights_scene) # flights scene got published

        self._activate_lg()

        self.assertEqual(self.attract_loop_controller.scene_timer <= 0, True) # scene timer is going down
        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 2) # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[1].message)['description'], 'rofl') # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[1].message)['windows'][0]['presentation_viewport'], 'mock3') # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[1].message)['windows'][1]['presentation_viewport'], 'mock2') # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[1].message)['windows'][2]['presentation_viewport'], 'mock1') # blank message was published because of 'go_blank'
        self.assertEqual(len(self.earth_query_publisher.published_messages), 1) # earth was stopped

        self._deactivate_lg()

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 3) # flights + blank + mplayer
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[2].message)['slug'], 'mplayer-two-different-instances') # blank message was published because of 'go_blank'
        self.assertEqual(len(self.attract_loop_controller.attract_loop_queue), 2)  # attract loop filled with new content again
        self.assertEqual(self.attract_loop_controller.scene_timer >= 9000, True) # scene timer is going down

        self._activate_lg()

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 4) # flights + blank + mplayer + blank
        self.assertEqual(len(self.earth_query_publisher.published_messages), 2) # earth was stopped
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[3].message)['description'], 'rofl') # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[3].message)['windows'][0]['presentation_viewport'], 'mock3') # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[3].message)['windows'][1]['presentation_viewport'], 'mock2') # blank message was published because of 'go_blank'
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[3].message)['windows'][2]['presentation_viewport'], 'mock1') # blank message was published because of 'go_blank'

    def test_2_entering_and_exiting_attract_loop_with_stop_playtour(self):
        self._init_mocks()
        self.stop_action = 'stop_playtour'
        self.attract_loop_controller = AttractLoop(api_proxy=self.mock_api,
                                              director_scene_publisher=self.mock_director_scene_publisher,
                                              director_presentation_publisher=self.mock_director_presentation_publisher,
                                              stop_action=self.stop_action,
                                              earth_query_publisher=self.earth_query_publisher,
                                              default_presentation=None)

        self.assertEqual(isinstance(self.attract_loop_controller, AttractLoop), True)
        self.assertEqual(self.attract_loop_controller.play_loop, False)
        self.assertEqual(self.attract_loop_controller.attract_loop_queue, [])

        rospy.loginfo("game is on - first wait for initialization and check whether attract loop was populated with scenes")
        rospy.sleep(2)
        self.assertEqual(len(self.attract_loop_controller.attract_loop_queue), 2) # two scenes waiting for playback
        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 0) # no scenes published yet

        self._deactivate_lg()

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 1) # one scene playing back right now (1000 seconds :))
        self.assertEqual(len(self.attract_loop_controller.attract_loop_queue), 1)  # the other one in the queue
        self.assertEqual(self.attract_loop_controller.attract_loop_queue[0]['scene']['slug'], self.mock_api.mplayer_scene['slug']) # mplayer scene is waiting for publication
        self.assertEqual(self.attract_loop_controller.attract_loop_queue[0]['scene']['description'], self.mock_api.mplayer_scene['description']) # mplayer scene again
        self.assertEqual(self.attract_loop_controller.scene_timer > 500, True) # scene timer should be sth lik 997 here
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[0].message), self.mock_api.flights_scene) # flights scene got published

        self._activate_lg()

        self.assertEqual(self.attract_loop_controller.scene_timer <= 0, True) # scene timer is going down
        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 1) # only playtour was published
        self.assertEqual(len(self.earth_query_publisher.published_messages), 1) # playtour was published

        self._deactivate_lg()

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 2) # flights + mplayer
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[1].message)['slug'], 'mplayer-two-different-instances')
        self.assertEqual(json.loads(self.mock_director_scene_publisher.published_scenes[0].message)['slug'], 'flights')
        self.assertEqual(len(self.attract_loop_controller.attract_loop_queue), 2)  # attract loop filled with new content again
        self.assertEqual(self.attract_loop_controller.scene_timer >= 9000, True) # scene timer is going down

        self._activate_lg()

        self.assertEqual(len(self.mock_director_scene_publisher.published_scenes), 2) # flights + mplayer
        self.assertEqual(len(self.earth_query_publisher.published_messages), 2) # earth was stopped


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAttractLoop)
