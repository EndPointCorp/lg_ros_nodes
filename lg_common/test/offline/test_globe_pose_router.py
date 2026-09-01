import importlib.util
import json
import os
import unittest


MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'src', 'lg_common',
    'globe_pose_router.py')
SPEC = importlib.util.spec_from_file_location('lg_common_globe_pose_router',
                                               MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
GlobePoseRouter = MODULE.GlobePoseRouter


class Value(object):
    pass


def message(x=10, y=20, source=''):
    msg = Value()
    msg.header = Value()
    msg.header.frame_id = source
    msg.pose = Value()
    msg.pose.position = Value()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation = Value()
    msg.pose.orientation.x = 30
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    return msg


class TestGlobePoseRouter(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.commands = {base: [] for base in MODULE.BASES}
        self.sync = {base: [] for base in MODULE.BASES}
        self.live = {base: [] for base in MODULE.BASES}
        self.stops = {base: 0 for base in MODULE.BASES}
        self.poses = []
        self.router = GlobePoseRouter(
            command_outputs={
                base: values.append for base, values in self.commands.items()
            },
            sync_outputs={
                base: values.append for base, values in self.sync.items()
            },
            live_outputs={
                base: values.append for base, values in self.live.items()
            },
            live_stop_outputs={
                base: self.stop_callback(base) for base in MODULE.BASES
            },
            pose_output=self.poses.append,
            clock=lambda: self.now,
        )

    def stop_callback(self, base):
        def stop():
            self.stops[base] += 1
        return stop

    def session(self, action, owner='screen-a', session='gesture-1'):
        value = Value()
        value.data = json.dumps({
            'action': action,
            'owner': owner,
            'session': session,
        })
        return value

    def test_routes_command_only_to_selected_base(self):
        self.router.select('cesium')
        self.assertTrue(self.router.handle_command(message()))
        self.assertEqual(1, len(self.commands['cesium']))
        self.assertEqual(0, len(self.commands['earth']))

    def test_rejects_non_owner_touchscreen_commands(self):
        self.router.set_owner('screen-a')
        self.assertFalse(self.router.handle_command(
            message(source='touchscreen:screen-b')))
        self.assertTrue(self.router.handle_command(
            message(source='touchscreen:screen-a')))

    def test_routes_live_session_and_rejects_stale_session(self):
        self.router.set_owner('screen-a')
        self.assertTrue(self.router.handle_control_session(
            self.session('begin')))
        self.assertTrue(self.router.handle_command(
            message(source='touchscreen:screen-a:gesture-1')))
        self.assertEqual(1, len(self.live['earth']))
        self.assertFalse(self.router.handle_command(
            message(source='touchscreen:screen-a:old-gesture')))

        self.assertTrue(self.router.handle_control_session(
            self.session('end')))
        self.assertEqual(0, self.stops['earth'])
        self.now += 0.2
        self.assertFalse(self.router.handle_command(
            message(source='touchscreen:screen-a:gesture-1')))

    def test_routes_final_pose_when_it_arrives_just_after_end(self):
        self.router.handle_control_session(self.session('begin'))
        self.router.handle_control_session(self.session('end'))
        self.assertTrue(self.router.handle_command(
            message(source='touchscreen:screen-a:gesture-1')))
        self.assertEqual(1, len(self.live['earth']))
        self.assertFalse(self.router.handle_command(
            message(source='touchscreen:screen-a:gesture-1')))

    def test_replays_pose_when_it_arrives_just_before_begin(self):
        early = message(
            x=42,
            source='touchscreen:screen-a:gesture-1',
        )
        self.assertFalse(self.router.handle_command(early))
        self.assertEqual([], self.live['earth'])
        self.assertTrue(self.router.handle_control_session(
            self.session('begin')))
        self.assertEqual(1, len(self.live['earth']))
        self.assertEqual(42, self.live['earth'][0].position.x)

    def test_different_begin_does_not_consume_buffered_pose(self):
        self.router.handle_command(message(
            source='touchscreen:screen-a:old-gesture',
        ))
        self.router.handle_control_session(self.session(
            'begin', session='new-gesture',
        ))
        self.assertEqual([], self.live['earth'])

    def test_new_owner_cancels_post_end_grace(self):
        self.router.handle_control_session(self.session('begin'))
        self.router.handle_control_session(self.session('end'))
        self.router.set_owner('screen-b')
        self.assertFalse(self.router.handle_command(
            message(source='touchscreen:screen-a:gesture-1')))

    def test_session_begin_atomically_claims_owner(self):
        self.router.set_owner('screen-b')
        self.assertTrue(self.router.handle_control_session(
            self.session('begin', owner='screen-a')))
        self.assertEqual('screen-a', self.router.owner)
        self.assertTrue(self.router.handle_command(
            message(source='touchscreen:screen-a:gesture-1')))

    def test_new_owner_stops_active_control(self):
        self.router.set_owner('screen-a')
        self.router.handle_control_session(self.session('begin'))
        self.router.set_owner('screen-b')
        self.assertEqual(1, self.stops['earth'])

    def test_only_selected_feedback_becomes_authoritative(self):
        self.assertFalse(self.router.handle_feedback('cesium', message()))
        self.assertEqual([], self.poses)
        self.assertTrue(self.router.handle_feedback('earth', message()))
        self.assertEqual(1, len(self.poses))
        self.assertEqual(20, self.poses[0].pose.position.x)
        self.assertEqual(10, self.poses[0].pose.position.y)
        self.assertEqual(0, len(self.sync['earth']))
        self.assertEqual(0, len(self.sync['cesium']))
        self.router.sync_inactive()
        self.assertEqual(1, len(self.sync['cesium']))
        self.assertEqual(1, len(self.sync['unreal']))

    def test_selection_replays_latest_pose_to_new_base(self):
        self.router.handle_feedback('earth', message())
        self.router.select('cesium')
        self.assertEqual(1, len(self.sync['cesium']))

    def test_normalizes_equivalent_angles_at_router_boundary(self):
        command = message()
        command.pose.orientation.z = 360
        command.pose.orientation.y = 360
        self.assertTrue(self.router.handle_command(command))
        self.assertEqual(0, self.commands['earth'][0].orientation.z)
        self.assertEqual(0, self.commands['earth'][0].orientation.y)

        feedback = message()
        feedback.pose.orientation.z = -10
        feedback.pose.orientation.y = 180
        self.assertTrue(self.router.handle_feedback('earth', feedback))
        self.assertEqual(350, self.poses[0].pose.orientation.z)
        self.assertEqual(-180, self.poses[0].pose.orientation.y)

if __name__ == '__main__':
    unittest.main()
