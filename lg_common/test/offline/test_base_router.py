import importlib.util
import os
import unittest


MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'src', 'lg_common', 'base_router.py')
SPEC = importlib.util.spec_from_file_location('lg_common_base_router', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
BaseRouter = MODULE.BaseRouter


class TestBaseRouter(unittest.TestCase):
    def setUp(self):
        self.changes = []
        self.router = BaseRouter(on_change=self.changes.append)

    def test_default_is_published(self):
        self.assertEqual('earth', self.router.selected)
        self.assertEqual(['earth'], self.changes)

    def test_explicit_base_scene_changes_selection(self):
        self.assertTrue(self.router.handle_scene({
            'windows': [{'activity': 'lg_cesium'}],
        }))
        self.assertEqual('cesium', self.router.selected)

    def test_overlay_and_cleanup_scenes_preserve_selection(self):
        self.router.select('cesium')
        self.assertFalse(self.router.handle_scene({'windows': []}))
        self.assertFalse(self.router.handle_scene({
            'windows': [{'activity': 'no_activity'}],
        }))
        self.assertEqual('cesium', self.router.selected)

    def test_internal_earth_scene_does_not_switch_from_cesium(self):
        self.router.select('cesium')
        self.router.handle_scene({
            'windows': [{'activity': 'earth', 'select_base': False}],
        })
        self.assertEqual('cesium', self.router.selected)

    def test_internal_window_does_not_mask_an_explicit_base(self):
        self.router.handle_scene({'windows': [
            {'activity': 'earth', 'select_base': False},
            {'activity': 'cesium'},
        ]})
        self.assertEqual('cesium', self.router.selected)

    def test_unreal_is_a_base_option(self):
        self.router.handle_scene({'windows': [{'activity': 'unreal'}]})
        self.assertEqual('unreal', self.router.selected)

    def test_invalid_direct_selection_is_rejected(self):
        with self.assertRaises(ValueError):
            self.router.select('stars')


if __name__ == '__main__':
    unittest.main()
