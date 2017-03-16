#!/usr/bin/env python

PKG = 'lg_nav_to_device'
NAME = 'test_background_stopper'

import sys
import rospy
import rostest
import unittest
from lg_nav_to_device import BackgroundStopper
from std_msgs.msg import String
from interactivespaces_msgs.msg import GenericMessage

DISABLED_ACTIVITY = 'disableme'
DISABLED_SLUG = 'disableme'


class MockWriter:
    def __init__(self):
        """ Here we assume that DeviceWriter inits with state = True """
        self.state = True


class TestBackgroundStopper(unittest.TestCase):
    def setUp(self):
        self.writer = MockWriter()
        self.stopper = BackgroundStopper([DISABLED_ACTIVITY], self.writer)

    def _scene_gen(self, slug, activity):
        director_msg = GenericMessage()
        director_msg.type = 'json'
        director_msg.message = """
                {
                      "description":"",
                      "duration":30,
                      "name":"lg_nav_to_device test",
                      "resource_uri":"/director_api/scene/lg_nav_to_device-test/",
                      "slug":"%s",
                      "windows":[
                        {
                        "activity":"%s",
                        "assets":["foo", "bar"],
                        "height":600,
                        "presentation_viewport":"center",
                        "width":800,
                        "x_coord":100,
                        "y_coord":100
                        }
                        ]
                    }
        """ % (slug, activity)
        return director_msg

    def test_init_state(self):
        self.assertTrue(self.writer.state)

    def test_inert_scene(self):
        scene = self._scene_gen('a_slug', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

    def test_disabled_activity(self):
        scene = self._scene_gen('a_slug', DISABLED_ACTIVITY)
        self.stopper.handle_scene(scene)

        self.assertFalse(self.writer.state)

    def test_disabled_slug_first(self):
        slug_msg = String(DISABLED_SLUG)
        self.stopper.handle_slug(slug_msg)

        self.test_init_state()

        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)

        self.assertFalse(self.writer.state)

    def test_disabled_slug_second(self):
        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

        slug_msg = String(DISABLED_SLUG)
        self.stopper.handle_slug(slug_msg)

        self.assertFalse(self.writer.state)

    def test_inert_slug_first(self):
        slug_msg = String('fubs')
        self.stopper.handle_slug(slug_msg)

        self.test_init_state()

        scene = self._scene_gen('bufs', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

    def test_inert_slug_second(self):
        scene = self._scene_gen('fubs', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

        slug_msg = String('bufs')
        self.stopper.handle_slug(slug_msg)

        self.assertTrue(self.writer.state)

    def test_next_scene(self):
        slug_msg = String(DISABLED_SLUG)
        self.stopper.handle_slug(slug_msg)

        self.test_init_state()

        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)

        self.assertFalse(self.writer.state)

        scene = self._scene_gen('fubs', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestBackgroundStopper, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
