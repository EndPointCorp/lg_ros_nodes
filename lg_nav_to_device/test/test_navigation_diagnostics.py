#!/usr/bin/env python3

import importlib.util
import os
import unittest


# Load the pure diagnostic module directly so this unit test can also run on a
# development machine without ROS message packages installed.
MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'src', 'lg_nav_to_device',
    'navigation_diagnostics.py',
)
SPEC = importlib.util.spec_from_file_location('navigation_diagnostics', MODULE_PATH)
DIAGNOSTICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIAGNOSTICS)
GateStateMirror = DIAGNOSTICS.GateStateMirror
NavigationStallDetector = DIAGNOSTICS.NavigationStallDetector


class TestGateStateMirror(unittest.TestCase):
    def test_models_last_callback_wins(self):
        gate = GateStateMirror(['cesium'])
        writes = gate.handle_scene({
            'slug': 'cesium-scene',
            'windows': [{'activity': 'cesium'}],
        })
        self.assertFalse(writes[-1]['enabled'])
        self.assertFalse(gate.enabled)

        writes = gate.handle_earth_state('VISIBLE')
        self.assertTrue(writes[-1]['enabled'])
        self.assertTrue(gate.enabled)
        self.assertEqual('/earth/state direct callback', gate.last_writer)

    def test_disabled_state_does_not_override_activity(self):
        gate = GateStateMirror(['cesium'])
        gate.handle_scene({
            'slug': 'cesium-scene',
            'windows': [{'activity': 'cesium'}],
        })
        writes = gate.handle_disabled_state('/streetview/state', 'HIDDEN')
        self.assertEqual([], writes)
        self.assertFalse(gate.enabled)


class TestNavigationStallDetector(unittest.TestCase):
    def setUp(self):
        self.detector = NavigationStallDetector(
            activity_window=1.0,
            stall_timeout=1.0,
            pose_message_timeout=1.0,
            ongoing_interval=10.0,
        )

    def _mixed_input(self, now):
        self.detector.record_signal('mixed', 0.5, now, 0.01)

    def test_reports_gate_disabled(self):
        self.detector.set_gate_enabled(False)
        self._mixed_input(0.0)
        self.assertEqual([], self.detector.tick(0.0))
        self._mixed_input(1.1)
        events = self.detector.tick(1.1)
        self.assertEqual('stall_started', events[0]['event'])
        self.assertEqual('routing_gate_disabled', events[0]['classification'])

    def test_reports_connected_device_without_output(self):
        self.detector.set_device_status('connected')
        self._mixed_input(0.0)
        self.detector.record_pose((1, 2, 3, 4, 5, 6), 0.0)
        self.detector.tick(0.0)
        self._mixed_input(1.1)
        self.detector.record_pose((1.1, 2, 3, 4, 5, 6), 1.1)
        events = self.detector.tick(1.1)
        self.assertEqual('virtual_device_no_output', events[0]['classification'])

    def test_reports_earth_not_responding_then_recovers(self):
        self.detector.set_device_status('connected')
        self.detector.record_pose((1, 2, 3, 4, 5, 6), 0.0)
        self._mixed_input(0.1)
        self.detector.record_virtual_event(100, 0.1, 2)
        self.detector.tick(0.1)

        self._mixed_input(1.2)
        self.detector.record_virtual_event(100, 1.2, 2)
        # Pose messages continue, but the values do not move.
        self.detector.record_pose((1, 2, 3, 4, 5, 6), 1.2)
        self.assertEqual([], self.detector.tick(1.2))

        self._mixed_input(2.3)
        self.detector.record_virtual_event(100, 2.3, 2)
        self.detector.record_pose((1, 2, 3, 4, 5, 6), 2.3)
        events = self.detector.tick(2.3)
        self.assertEqual(
            'earth_not_responding_to_virtual_device',
            events[0]['classification'],
        )

        self._mixed_input(2.4)
        self.detector.record_virtual_event(100, 2.4, 2)
        self.detector.record_pose((1.1, 2, 3, 4, 5, 6), 2.4)
        events = self.detector.tick(2.4)
        self.assertEqual('stall_recovered', events[0]['event'])


if __name__ == '__main__':
    unittest.main()
