import importlib.util
import os
import unittest


MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'src', 'lg_earth', 'scene_assets.py')
SPEC = importlib.util.spec_from_file_location('lg_earth_scene_assets', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
assets_for_renderer = MODULE.assets_for_renderer
is_base_only_scene = MODULE.is_base_only_scene
is_kml_asset = MODULE.is_kml_asset


class TestSceneAssets(unittest.TestCase):
    def test_kml_detection_uses_url_path(self):
        self.assertTrue(is_kml_asset('https://example/a.KMZ?token=1'))
        self.assertFalse(is_kml_asset('https://example/a.yaml?kml=yes'))

    def test_earth_gets_native_assets_and_shared_globe_kml(self):
        scene = {'windows': [
            {'activity': 'earth', 'presentation_viewport': 'center',
             'assets': ['earth.kml', 'earth.data']},
            {'activity': 'cesium', 'presentation_viewport': 'center',
             'assets': ['shared.kmz', 'imagery.yaml', 'earth.kml']},
            {'activity': 'unreal', 'presentation_viewport': 'center',
             'assets': ['future.kml']},
            {'activity': 'cesium', 'presentation_viewport': 'center',
             'assets': ['internal-sync.kml'], 'select_base': False},
            {'activity': 'cesium', 'presentation_viewport': 'left_one',
             'assets': ['wrong-screen.kml']},
        ]}
        self.assertEqual(
            ['earth.kml', 'earth.data', 'shared.kmz', 'future.kml'],
            assets_for_renderer(scene, 'center', 'earth'))

    def test_base_only_scene_markers_are_content_neutral(self):
        self.assertTrue(is_base_only_scene({'slug': 'open-cesium'}))
        self.assertTrue(is_base_only_scene({'windows': [{
            'activity_config': {'force_touchscreen_tab': 'free_flight'},
        }]}))
        self.assertFalse(is_base_only_scene({'slug': 'presentation'}))

    def test_center_assets_are_available_to_a_solo_wall(self):
        scene = {'windows': [
            {'activity': 'cesium', 'presentation_viewport': 'center',
             'assets': ['shared.kmz']},
        ]}
        self.assertEqual(
            ['shared.kmz'], assets_for_renderer(scene, 'wall_a', 'earth'))

    def test_wildcard_assets_are_available_to_every_wall(self):
        scene = {'windows': [
            {'activity': 'unreal', 'presentation_viewport': '*',
             'assets': ['shared.kml']},
        ]}
        self.assertEqual(
            ['shared.kml'], assets_for_renderer(scene, 'right_one', 'earth'))


if __name__ == '__main__':
    unittest.main()
