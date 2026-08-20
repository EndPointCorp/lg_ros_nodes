import importlib.util
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
    return msg


class TestGlobePoseRouter(unittest.TestCase):
    def setUp(self):
        self.commands = {base: [] for base in MODULE.BASES}
        self.sync = {base: [] for base in MODULE.BASES}
        self.poses = []
        self.router = GlobePoseRouter(
            command_outputs={
                base: values.append for base, values in self.commands.items()
            },
            sync_outputs={
                base: values.append for base, values in self.sync.items()
            },
            pose_output=self.poses.append,
        )

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

    def test_only_selected_feedback_becomes_authoritative(self):
        self.assertFalse(self.router.handle_feedback('cesium', message()))
        self.assertEqual([], self.poses)
        self.assertTrue(self.router.handle_feedback('earth', message()))
        self.assertEqual(1, len(self.poses))
        self.assertEqual(20, self.poses[0].pose.position.x)
        self.assertEqual(10, self.poses[0].pose.position.y)
        self.assertEqual(0, len(self.sync['earth']))
        self.assertEqual(1, len(self.sync['cesium']))
        self.assertEqual(1, len(self.sync['unreal']))

    def test_selection_replays_latest_pose_to_new_base(self):
        self.router.handle_feedback('earth', message())
        self.router.select('cesium')
        self.assertEqual(2, len(self.sync['cesium']))


if __name__ == '__main__':
    unittest.main()
