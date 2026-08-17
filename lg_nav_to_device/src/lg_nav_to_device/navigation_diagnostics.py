"""State tracking and stall classification for SpaceNav-to-Earth diagnostics.

This module intentionally has no ROS or evdev imports so its ordering and
classification behavior can be tested offline.
"""

import copy
import threading


VISIBLE = 'VISIBLE'


class GateStateMirror(object):
    """Mirror the composed routing decision made by BackgroundStopper."""

    def __init__(self, disable_activities):
        self.disable_activities = set(disable_activities)
        self.enabled = True
        self.last_writer = 'initial default'
        self.earth_state = None
        self.earth_visible = True
        self.current_scene_slug = ''
        self.disabled_scene_slug = ''
        self.activity_disabled = False
        self.slug_disabled = False
        self.states = {}

    def _write(self, enabled, writer, details=None):
        previous = self.enabled
        self.enabled = bool(enabled)
        self.last_writer = writer
        return {
            'writer': writer,
            'previous_enabled': previous,
            'enabled': self.enabled,
            'details': details or {},
        }

    def _recompute(self, writer, details=None):
        enabled = (
            self.earth_visible and
            not self.activity_disabled and
            not self.slug_disabled and
            not any(self.states.values())
        )
        return [self._write(enabled, writer, details)]

    def handle_earth_state(self, state):
        self.earth_state = state
        self.earth_visible = state == VISIBLE
        return self._recompute(
            '/earth/state composed callback',
            {'earth_state': state},
        )

    def handle_scene(self, scene):
        # BackgroundStopper clears disabled-state observations on every scene.
        self.states = {}
        self.current_scene_slug = scene.get('slug')
        self.slug_disabled = bool(
            self.current_scene_slug and
            self.disabled_scene_slug and
            self.current_scene_slug == self.disabled_scene_slug
        )

        activities = set(
            window.get('activity')
            for window in scene.get('windows', [])
            if window.get('activity') is not None
        )
        matching = sorted(activities.intersection(self.disable_activities))
        self.activity_disabled = bool(matching)
        return self._recompute('/director/scene composed callback', {
            'disabled_activities_present': matching,
            'current_scene_slug': self.current_scene_slug,
            'disabled_scene_slug': self.disabled_scene_slug,
        })

    def handle_slug(self, slug):
        self.disabled_scene_slug = slug
        self.slug_disabled = bool(
            self.current_scene_slug and
            self.disabled_scene_slug and
            self.current_scene_slug == self.disabled_scene_slug
        )
        return self._recompute(
            '/earth/disable_nav_for_scene_slug composed callback', {
                'current_scene_slug': self.current_scene_slug,
                'disabled_scene_slug': self.disabled_scene_slug,
            }
        )

    def handle_disabled_state(self, topic, state):
        self.states[topic] = state == VISIBLE
        visible_topics = sorted(
            topic_name for topic_name, visible in self.states.items() if visible
        )
        return self._recompute(
            '{} composed disabled-state callback'.format(topic),
            {'state': state, 'visible_disabled_state_topics': visible_topics},
        )

    def snapshot(self):
        return {
            'modeled_enabled': self.enabled,
            'last_writer': self.last_writer,
            'earth_state': self.earth_state,
            'earth_visible': self.earth_visible,
            'current_scene_slug': self.current_scene_slug,
            'disabled_scene_slug': self.disabled_scene_slug,
            'activity_disabled': self.activity_disabled,
            'slug_disabled': self.slug_disabled,
            'disabled_states_visible': sorted(
                topic for topic, visible in self.states.items() if visible
            ),
        }


class NavigationStallDetector(object):
    """Correlate navigation stages and identify persistent pipeline stalls."""

    STAGES = ('physical', 'wrapper', 'touch', 'pointer', 'mixed')

    def __init__(self, activity_window=0.75, stall_timeout=1.5,
                 pose_message_timeout=1.5, ongoing_interval=5.0,
                 pose_latlon_epsilon=1e-8, pose_altitude_epsilon=0.05,
                 pose_angle_epsilon=1e-3):
        self.activity_window = activity_window
        self.stall_timeout = stall_timeout
        self.pose_message_timeout = pose_message_timeout
        self.ongoing_interval = ongoing_interval
        self.pose_latlon_epsilon = pose_latlon_epsilon
        self.pose_altitude_epsilon = pose_altitude_epsilon
        self.pose_angle_epsilon = pose_angle_epsilon

        self._lock = threading.RLock()
        self.signals = {}
        for stage in self.STAGES:
            self.signals[stage] = {
                'last_message': None,
                'last_active': None,
                'magnitude': 0.0,
            }

        self.gate_enabled = True
        self.device_status = 'unknown'
        self.device_details = {}
        self.virtual_last_message = None
        self.virtual_last_active = None
        self.virtual_value = 0

        self.pose = None
        self.pose_motion_reference = None
        self.pose_last_message = None
        self.pose_last_motion = None

        self._candidate = None
        self._candidate_since = None
        self._active_stall = None
        self._active_stall_since = None
        self._last_ongoing = None

    def record_signal(self, stage, magnitude, now, threshold):
        with self._lock:
            signal = self.signals[stage]
            signal['last_message'] = now
            signal['magnitude'] = magnitude
            if magnitude >= threshold:
                signal['last_active'] = now

    def set_gate_enabled(self, enabled):
        with self._lock:
            self.gate_enabled = bool(enabled)

    def set_device_status(self, status, details=None):
        with self._lock:
            self.device_status = status
            self.device_details = details or {}

    def record_virtual_event(self, value, now, threshold):
        with self._lock:
            self.virtual_last_message = now
            self.virtual_value = value
            if abs(value) >= threshold:
                self.virtual_last_active = now

    def record_pose(self, pose, now):
        """Record (lat, lon, altitude, tilt, roll, heading)."""
        with self._lock:
            self.pose = tuple(pose)
            self.pose_last_message = now
            if self.pose_motion_reference is None:
                moved = True
            else:
                old = self.pose_motion_reference
                moved = (
                    abs(pose[0] - old[0]) >= self.pose_latlon_epsilon or
                    abs(pose[1] - old[1]) >= self.pose_latlon_epsilon or
                    abs(pose[2] - old[2]) >= self.pose_altitude_epsilon or
                    abs(pose[3] - old[3]) >= self.pose_angle_epsilon or
                    abs(pose[4] - old[4]) >= self.pose_angle_epsilon or
                    abs(pose[5] - old[5]) >= self.pose_angle_epsilon
                )
            if moved:
                self.pose_motion_reference = tuple(pose)
                self.pose_last_motion = now

    @staticmethod
    def _age(timestamp, now):
        if timestamp is None:
            return None
        return max(0.0, now - timestamp)

    def _is_recent(self, timestamp, now, window=None):
        if timestamp is None:
            return False
        if window is None:
            window = self.activity_window
        return now - timestamp <= window

    def active_signals(self, now):
        with self._lock:
            return {
                stage: self._is_recent(signal['last_active'], now)
                for stage, signal in self.signals.items()
            }

    def _classify(self, now):
        active = self.active_signals(now)
        if active['physical'] and not active['wrapper']:
            return 'physical_to_wrapper_gap'
        if active['wrapper'] and not active['mixed']:
            return 'wrapper_to_mixer_gap'
        if not active['mixed']:
            return None
        if not self.gate_enabled:
            return 'routing_gate_disabled'
        if self.device_status == 'disconnected':
            return 'virtual_device_disconnected'

        pose_recent = self._is_recent(
            self.pose_last_message, now, self.pose_message_timeout
        )
        pose_moving = self._is_recent(self.pose_last_motion, now)
        virtual_active = self._is_recent(self.virtual_last_active, now)

        if self.device_status == 'connected' and not virtual_active:
            return 'virtual_device_no_output'
        if not pose_recent:
            if self.device_status == 'unavailable':
                return 'earth_pose_missing_virtual_unobservable'
            return 'earth_pose_feedback_missing'
        if not pose_moving:
            if self.device_status == 'unavailable':
                return 'earth_static_virtual_unobservable'
            if virtual_active:
                return 'earth_not_responding_to_virtual_device'
            return 'earth_not_moving'
        return None

    def tick(self, now):
        """Return zero or more stall transition dictionaries."""
        with self._lock:
            classification = self._classify(now)
            events = []

            if classification != self._candidate:
                self._candidate = classification
                self._candidate_since = now if classification else None

            if classification is None:
                if self._active_stall is not None:
                    events.append({
                        'event': 'stall_recovered',
                        'classification': self._active_stall,
                        'duration': now - self._active_stall_since,
                    })
                self._active_stall = None
                self._active_stall_since = None
                self._last_ongoing = None
                return events

            candidate_age = now - self._candidate_since
            if candidate_age < self.stall_timeout:
                return events

            if self._active_stall != classification:
                if self._active_stall is not None:
                    events.append({
                        'event': 'stall_changed',
                        'classification': self._active_stall,
                        'new_classification': classification,
                        'duration': now - self._active_stall_since,
                    })
                self._active_stall = classification
                self._active_stall_since = self._candidate_since
                self._last_ongoing = now
                events.append({
                    'event': 'stall_started',
                    'classification': classification,
                    'candidate_duration': candidate_age,
                })
            elif now - self._last_ongoing >= self.ongoing_interval:
                self._last_ongoing = now
                events.append({
                    'event': 'stall_ongoing',
                    'classification': classification,
                    'duration': now - self._active_stall_since,
                })
            return events

    def snapshot(self, now):
        with self._lock:
            signals = {}
            for stage, signal in self.signals.items():
                signals[stage] = {
                    'active': self._is_recent(signal['last_active'], now),
                    'message_age': self._age(signal['last_message'], now),
                    'active_age': self._age(signal['last_active'], now),
                    'last_magnitude': signal['magnitude'],
                }
            return {
                'signals': signals,
                'gate_enabled': self.gate_enabled,
                'virtual_device': {
                    'status': self.device_status,
                    'details': copy.deepcopy(self.device_details),
                    'message_age': self._age(self.virtual_last_message, now),
                    'active_age': self._age(self.virtual_last_active, now),
                    'last_value': self.virtual_value,
                },
                'earth': {
                    'pose_message_age': self._age(self.pose_last_message, now),
                    'pose_motion_age': self._age(self.pose_last_motion, now),
                    'pose': self.pose,
                },
                'candidate_stall': self._candidate,
                'active_stall': self._active_stall,
            }
