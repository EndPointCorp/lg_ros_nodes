#!/usr/bin/env python3
PKG = 'lg_common'
NAME = 'test_managed_application'

import gc
import os
import unittest
import weakref

from lg_msg_defs.msg import ApplicationState
from appctl_support import ProcController
from lg_common import ManagedApplication, ManagedWindow

TEST_CMD = ['/usr/bin/python3']


class MockWindow(ManagedWindow):
    def __init__(self, *args, **kwargs):
        self.converge_count = 0
        super(MockWindow, self).__init__(*args, **kwargs)

    def converge(self):
        self.converge_count += 1


class MockController(ProcController):
    def __init__(self, *args, **kwargs):
        super(MockController, self).__init__(*args, **kwargs)
        self.start_count = 0
        self.stop_count = 0

    def start(self):
        self.start_count += 1

    def stop(self):
        self.stop_count += 1


class TestManagedApplication(unittest.TestCase):
    def setUp(self):
        window = MockWindow(w_instance='NULL', visible=False)
        self.app = ManagedApplication(cmd=TEST_CMD, window=window)

    def test_init(self):
        self.app.proc = MockController(TEST_CMD)
        self.assertEqual(ApplicationState.STOPPED, self.app.get_state())
        self.assertEqual(0, self.app.window.converge_count)
        self.assertEqual(0, self.app.proc.start_count)
        self.assertEqual(0, self.app.proc.stop_count)

    def test_set_state_suspended(self):
        self.app.proc = MockController(TEST_CMD)
        self.app.set_state(ApplicationState.SUSPENDED)
        self.assertFalse(self.app.window.is_visible)
        self.assertEqual(1, self.app.window.converge_count)
        self.assertEqual(1, self.app.proc.start_count)
        self.assertEqual(0, self.app.proc.stop_count)

        self.app.set_state(ApplicationState.STOPPED)
        self.assertFalse(self.app.window.is_visible)
        self.assertEqual(1, self.app.window.converge_count)
        self.assertEqual(1, self.app.proc.start_count)
        self.assertEqual(1, self.app.proc.stop_count)

    def test_set_state_hidden(self):
        self.app.proc = MockController(TEST_CMD)
        self.app.set_state(ApplicationState.HIDDEN)
        self.assertFalse(self.app.window.is_visible)
        self.assertEqual(1, self.app.window.converge_count)
        self.assertEqual(1, self.app.proc.start_count)
        self.assertEqual(0, self.app.proc.stop_count)

        self.app.set_state(ApplicationState.STOPPED)
        self.assertFalse(self.app.window.is_visible)
        self.assertEqual(1, self.app.window.converge_count)
        self.assertEqual(1, self.app.proc.start_count)
        self.assertEqual(1, self.app.proc.stop_count)

    def test_set_state_visible(self):
        self.app.proc = MockController(TEST_CMD)
        self.app.set_state(ApplicationState.VISIBLE)
        self.assertTrue(self.app.window.is_visible)
        self.assertEqual(1, self.app.window.converge_count)
        self.assertEqual(1, self.app.proc.start_count)
        self.assertEqual(0, self.app.proc.stop_count)

        self.app.set_state(ApplicationState.STOPPED)
        self.assertFalse(self.app.window.is_visible)
        self.assertEqual(1, self.app.window.converge_count)
        self.assertEqual(1, self.app.proc.start_count)
        self.assertEqual(1, self.app.proc.stop_count)


class TestCleanup(unittest.TestCase):
    def test_cleanup(self):
        app = ManagedApplication(cmd=TEST_CMD)
        app_ref = weakref.ref(app)
        app.close()
        app = None
        gc.collect()
        self.assertIsNone(app_ref())


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestManagedApplication)
    rostest.rosrun(PKG, NAME, TestCleanup)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
