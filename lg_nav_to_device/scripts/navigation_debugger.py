#!/usr/bin/env python3
"""Passive diagnostic node for intermittent SpaceNav-to-Earth stalls."""

import datetime
import json
import os
import select
import threading
import time

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from interactivespaces_msgs.msg import GenericMessage
    from lg_msg_defs.msg import ApplicationState
    from std_msgs.msg import String
except ImportError:
    # Current VPROS images provide a rospy-compatible interface and generated
    # message models under visionport.vpros.
    import visionport.vpros as rospy
    from visionport.vpros.models.geometry_msgs.msg import PoseStamped, Twist
    from visionport.vpros.models.interactivespaces_msgs.msg import GenericMessage
    from visionport.vpros.models.lg_msg_defs.msg import ApplicationState
    from visionport.vpros.models.std_msgs.msg import String

try:
    from lg_nav_to_device.navigation_diagnostics import (
        GateStateMirror,
        NavigationStallDetector,
    )
except ImportError:
    # Allow this script and navigation_diagnostics.py to be copied together
    # and run directly on a live system without rebuilding its catkin space.
    from navigation_diagnostics import GateStateMirror, NavigationStallDetector


DEFAULT_DISABLE_ACTIVITIES = 'cesium,unity,sketchfab,streetview,panovideo'
DEFAULT_DISABLE_STATES = '/streetview/state'


def split_param(value):
    if isinstance(value, list):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(',') if item.strip()]


def twist_magnitude(msg):
    values = (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    )
    return max(abs(value) for value in values)


class JsonEventLogger(object):
    def __init__(self, path):
        self.path = path
        parent = os.path.dirname(path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        self._file = open(path, 'a', buffering=1)
        self._lock = threading.Lock()

    def emit(self, event, level='info', **fields):
        record = {
            'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'monotonic': time.monotonic(),
            'event': event,
        }
        record.update(fields)
        line = json.dumps(record, sort_keys=True, separators=(',', ':'))
        with self._lock:
            self._file.write(line + '\n')
        message = '{} {}'.format(event, json.dumps(fields, sort_keys=True))
        if level == 'warn':
            rospy.logwarn(message)
        elif level == 'error':
            rospy.logerr(message)
        else:
            rospy.loginfo(message)

    def close(self):
        with self._lock:
            self._file.close()


class EvdevMonitor(object):
    """Observe a virtual input device without grabbing it."""

    def __init__(self, path, detector, logger, event_threshold):
        self.path = path
        self.detector = detector
        self.logger = logger
        self.event_threshold = event_threshold
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._last_status = None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _set_status(self, status, **details):
        self.detector.set_device_status(status, details)
        status_key = (status, json.dumps(details, sort_keys=True))
        if status_key != self._last_status:
            self._last_status = status_key
            level = 'warn' if status != 'connected' else 'info'
            self.logger.emit(
                'virtual_device_status', level=level,
                status=status, details=details,
            )

    def _run(self):
        try:
            from evdev import InputDevice, ecodes
        except ImportError as exc:
            self._set_status('unavailable', error='evdev import failed: {}'.format(exc))
            return

        device = None
        while not self._stop.is_set() and not rospy.is_shutdown():
            if device is None:
                try:
                    stat_result = os.stat(self.path)
                    device = InputDevice(self.path)
                    self._set_status(
                        'connected', path=self.path, name=device.name,
                        inode=stat_result.st_ino, rdev=stat_result.st_rdev,
                    )
                except PermissionError as exc:
                    self._set_status('unavailable', path=self.path, error=str(exc))
                    self._stop.wait(2.0)
                    continue
                except (FileNotFoundError, OSError) as exc:
                    self._set_status('disconnected', path=self.path, error=str(exc))
                    self._stop.wait(0.5)
                    continue

            try:
                readable, _, _ = select.select([device.fd], [], [], 0.25)
                if not readable:
                    continue
                for event in device.read():
                    if event.type == ecodes.EV_REL:
                        self.detector.record_virtual_event(
                            event.value, time.monotonic(), self.event_threshold
                        )
            except (OSError, ValueError) as exc:
                try:
                    device.close()
                except Exception:
                    pass
                device = None
                self._set_status('disconnected', path=self.path, error=str(exc))

        if device is not None:
            device.close()


class NavigationDebugger(object):
    def __init__(self):
        self.input_threshold = float(rospy.get_param('~input_threshold', 0.01))
        self.virtual_threshold = int(rospy.get_param('~virtual_event_threshold', 2))
        self.summary_interval = float(rospy.get_param('~summary_interval', 10.0))
        self.log_file = rospy.get_param(
            '~log_file', '/tmp/lg_navigation_debug.jsonl'
        )
        self.logger = JsonEventLogger(self.log_file)

        self.writer_node = rospy.get_param('~writer_node', '/spacenav_emulator')
        disable_activities = split_param(self._routing_param(
            'disable_activities', DEFAULT_DISABLE_ACTIVITIES
        ))
        self.disable_states = split_param(self._routing_param(
            'disable_states', DEFAULT_DISABLE_STATES
        ))
        self.gate = GateStateMirror(disable_activities)
        self.detector = NavigationStallDetector(
            activity_window=float(rospy.get_param('~activity_window', 0.75)),
            stall_timeout=float(rospy.get_param('~stall_timeout', 1.5)),
            pose_message_timeout=float(rospy.get_param(
                '~pose_message_timeout', 1.5
            )),
            ongoing_interval=float(rospy.get_param('~ongoing_interval', 5.0)),
            pose_latlon_epsilon=float(rospy.get_param(
                '~pose_latlon_epsilon', 1e-8
            )),
            pose_altitude_epsilon=float(rospy.get_param(
                '~pose_altitude_epsilon', 0.05
            )),
            pose_angle_epsilon=float(rospy.get_param(
                '~pose_angle_epsilon', 1e-3
            )),
        )

        self._activity_state = {}
        self._last_summary = 0.0
        self._subscribe_twist('physical', rospy.get_param(
            '~physical_topic', '/spacenav/twist'
        ))
        self._subscribe_twist('wrapper', rospy.get_param(
            '~wrapper_topic', '/spacenav_wrapper/twist'
        ))
        self._subscribe_twist('touch', rospy.get_param(
            '~touch_topic', '/navtransform/twist'
        ))
        self._subscribe_twist('pointer', rospy.get_param(
            '~pointer_topic', '/lg_pointer/twist'
        ))
        self._subscribe_twist('mixed', rospy.get_param(
            '~mixed_topic', '/lg_twister/twist'
        ))

        rospy.Subscriber('/earth/pose', PoseStamped, self._handle_pose)
        rospy.Subscriber('/earth/state', ApplicationState, self._handle_earth_state)
        rospy.Subscriber('/director/scene', GenericMessage, self._handle_scene)
        rospy.Subscriber(
            '/earth/disable_nav_for_scene_slug', String, self._handle_slug
        )
        for state_topic in self.disable_states:
            rospy.Subscriber(
                state_topic, ApplicationState,
                self._make_disabled_state_handler(state_topic)
            )

        self.device_monitor = EvdevMonitor(
            rospy.get_param('~device_path', '/dev/input/v_spacenav'),
            self.detector, self.logger, self.virtual_threshold,
        )
        self.device_monitor.start()
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.on_shutdown(self.shutdown)

        self.logger.emit(
            'navigation_debugger_started', log_file=self.log_file,
            input_threshold=self.input_threshold,
            virtual_event_threshold=self.virtual_threshold,
            disable_activities=disable_activities,
            disable_states=self.disable_states,
            writer_node=self.writer_node,
        )

    def _routing_param(self, name, default):
        private_name = '~{}'.format(name)
        writer_name = '{}/{}'.format(self.writer_node.rstrip('/'), name)
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)
        if rospy.has_param(writer_name):
            return rospy.get_param(writer_name)
        return default

    def _subscribe_twist(self, stage, topic):
        rospy.Subscriber(topic, Twist, self._make_twist_handler(stage), queue_size=50)

    def _make_twist_handler(self, stage):
        def handler(msg):
            self.detector.record_signal(
                stage, twist_magnitude(msg), time.monotonic(), self.input_threshold
            )
        return handler

    def _handle_pose(self, msg):
        pose = msg.pose
        # /earth/pose uses the legacy x=latitude, y=longitude convention.
        self.detector.record_pose((
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
        ), time.monotonic())

    def _apply_gate_writes(self, writes):
        for write in writes:
            self.detector.set_gate_enabled(write['enabled'])
            self.logger.emit('modeled_gate_write', **write)

    def _handle_earth_state(self, msg):
        self.logger.emit('earth_state', state=msg.state)
        self._apply_gate_writes(self.gate.handle_earth_state(msg.state))

    def _handle_scene(self, msg):
        try:
            scene = json.loads(msg.message)
        except Exception as exc:
            self.logger.emit(
                'director_scene_parse_error', level='warn', error=str(exc),
                raw_message=msg.message,
            )
            return
        activities = sorted(
            window.get('activity') for window in scene.get('windows', [])
            if window.get('activity') is not None
        )
        self.logger.emit(
            'director_scene', slug=scene.get('slug'), activities=activities
        )
        self._apply_gate_writes(self.gate.handle_scene(scene))

    def _handle_slug(self, msg):
        self.logger.emit('disabled_scene_slug', slug=msg.data)
        self._apply_gate_writes(self.gate.handle_slug(msg.data))

    def _make_disabled_state_handler(self, topic):
        def handler(msg):
            self.logger.emit(
                'disabled_application_state', topic=topic, state=msg.state
            )
            self._apply_gate_writes(
                self.gate.handle_disabled_state(topic, msg.state)
            )
        return handler

    def _snapshot(self, now):
        snapshot = self.detector.snapshot(now)
        snapshot['gate'] = self.gate.snapshot()
        return snapshot

    def _tick(self, _event):
        now = time.monotonic()
        active = self.detector.active_signals(now)
        for stage, is_active in active.items():
            if self._activity_state.get(stage) != is_active:
                self._activity_state[stage] = is_active
                self.logger.emit(
                    'pipeline_activity', stage=stage, active=is_active
                )

        for transition in self.detector.tick(now):
            level = 'info' if transition['event'] == 'stall_recovered' else 'warn'
            event_name = transition.pop('event')
            self.logger.emit(
                event_name, level=level, snapshot=self._snapshot(now), **transition
            )

        if now - self._last_summary >= self.summary_interval:
            self._last_summary = now
            self.logger.emit('status_summary', snapshot=self._snapshot(now))

    def shutdown(self):
        if getattr(self, 'device_monitor', None) is not None:
            self.device_monitor.stop()
            self.device_monitor = None
        if getattr(self, 'logger', None) is not None:
            self.logger.emit('navigation_debugger_stopped')
            self.logger.close()
            self.logger = None


def main():
    rospy.init_node('navigation_debugger')
    debugger = NavigationDebugger()
    rospy.loginfo('Writing navigation diagnostics to {}'.format(debugger.log_file))
    rospy.spin()


if __name__ == '__main__':
    main()
