#!/usr/bin/env python

PKG = 'lg_nav_to_device'
NAME = 'test_background_stopper'

import sys
import rospy
import rostest
import unittest
from lg_nav_to_device import BackgroundStopper
from std_msgs.msg import String
from lg_common.msg import ApplicationState
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

        # shortcuts for messages
        self.disabled_slug_msg = String(DISABLED_SLUG)
        self.visible_msg = ApplicationState(ApplicationState.VISIBLE)
        self.hidden_msg = ApplicationState(ApplicationState.HIDDEN)
        self.stopped_msg = ApplicationState(ApplicationState.STOPPED)

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
        """
        Test noop.
        """
        self.assertTrue(self.writer.state)

    def test_inert_scene(self):
        """
        Test a normal scene with no disabled activities.
        """
        scene = self._scene_gen('a_slug', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

    def test_disabled_activity(self):
        """
        Test a scene with a disabled activity.
        """
        scene = self._scene_gen('a_slug', DISABLED_ACTIVITY)
        self.stopper.handle_scene(scene)

        self.assertFalse(self.writer.state)

    def test_disabled_slug_first(self):
        """
        Test a disabled slug before a new scene.
        """
        self.stopper.handle_slug(self.disabled_slug_msg)

        self.test_init_state()

        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)

        self.assertFalse(self.writer.state)

    def test_disabled_slug_second(self):
        """
        Test a disabled slug after a new scene.
        """
        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

        slug_msg = String(DISABLED_SLUG)
        self.stopper.handle_slug(slug_msg)

        self.assertFalse(self.writer.state)

    def test_inert_slug_first(self):
        """
        Test an inert slug disable before a new scene.
        """
        slug_msg = String('fubs')
        self.stopper.handle_slug(slug_msg)

        self.test_init_state()

        scene = self._scene_gen('bufs', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

    def test_inert_slug_second(self):
        """
        Test an inert slug disable after a new scene.
        """
        scene = self._scene_gen('fubs', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

        slug_msg = String('bufs')
        self.stopper.handle_slug(slug_msg)

        self.assertTrue(self.writer.state)

    def test_disabled_slug_next_scene(self):
        """
        Clear disabled slug on new scene.
        """
        self.stopper.handle_slug(self.disabled_slug_msg)

        self.test_init_state()

        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)

        self.assertFalse(self.writer.state)

        scene = self._scene_gen('fubs', 'cats')
        self.stopper.handle_scene(scene)

        self.assertTrue(self.writer.state)

    def test_disabled_state_hidden(self):
        """
        Recover from disabled state on HIDDEN.
        """
        self.stopper.handle_disabled_state('/foo', self.visible_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.hidden_msg)
        self.assertTrue(self.writer.state)

    def test_disabled_state_stopped(self):
        """
        Recover from disabled state on STOPPED.
        """
        self.stopper.handle_disabled_state('/foo', self.visible_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.stopped_msg)
        self.assertTrue(self.writer.state)

    def test_multiple_disabled_states(self):
        """
        Make sure that when there are multiple disabled state topics,
        we are disabled when any of them are visible.
        """
        self.stopper.handle_disabled_state('/foo', self.visible_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/bar', self.visible_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.hidden_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/bar', self.hidden_msg)
        self.assertTrue(self.writer.state)

    def test_disabled_state_cleared_on_new_scene(self):
        """
        Clear disabled states completely upon new scene.
        """
        self.stopper.handle_disabled_state('/foo', self.visible_msg)
        self.assertFalse(self.writer.state)

        scene = self._scene_gen('a_slug', 'cats')
        self.stopper.handle_scene(scene)
        self.assertTrue(self.writer.state)

    def test_disabled_state_after_disabled_activity(self):
        """
        Disabled states are second class to disabled activities.
        """
        scene = self._scene_gen('a_slug', DISABLED_ACTIVITY)
        self.stopper.handle_scene(scene)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.visible_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.hidden_msg)
        self.assertFalse(self.writer.state)

    def test_disabled_state_after_disabled_slug(self):
        """
        Disabled states are second class to disabled slugs.
        """
        self.stopper.handle_slug(self.disabled_slug_msg)

        scene = self._scene_gen(DISABLED_SLUG, 'cats')
        self.stopper.handle_scene(scene)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.visible_msg)
        self.assertFalse(self.writer.state)

        self.stopper.handle_disabled_state('/foo', self.hidden_msg)
        self.assertFalse(self.writer.state)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestBackgroundStopper, sys.argv)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
