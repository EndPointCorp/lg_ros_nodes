import importlib.util
import os
import unittest
import xml.etree.ElementTree as ET


MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'src', 'lg_earth', 'tour.py')
SPEC = importlib.util.spec_from_file_location('lg_earth_tour', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
attach_center_tour = MODULE.attach_center_tour
build_tour_kml = MODULE.build_tour_kml


CAMERA = (
    '<Camera><longitude>-122</longitude><latitude>37</latitude>'
    '<altitude>1500</altitude></Camera>'
)


class TestTour(unittest.TestCase):
    def test_builds_zero_duration_self_playing_tour(self):
        kml = build_tour_kml(CAMERA, 'test-tour')
        root = ET.fromstring(kml)
        namespace = {
            'kml': 'http://www.opengis.net/kml/2.2',
            'gx': 'http://www.google.com/kml/ext/2.2',
        }
        self.assertEqual('0', root.find('.//gx:duration', namespace).text)
        href = root.find('.//kml:href', namespace).text
        self.assertEqual(
            'http://localhost:8765/query.html?query=playtour=test-tour', href)
        children = list(root.find('kml:Document', namespace))
        self.assertTrue(children[0].tag.endswith('Tour'))
        self.assertTrue(children[1].tag.endswith('NetworkLink'))

    def test_attaches_only_to_center_earth_and_replaces_prior_tour(self):
        scene = {'windows': [
            {'activity': 'earth', 'presentation_viewport': 'left_one',
             'assets': ['left.kml']},
            {'activity': 'earth', 'presentation_viewport': 'center',
             'assets': ['keep.kml', 'http://42-a:18112/old.kml']},
            {'activity': 'earth', 'presentation_viewport': 'right_one',
             'assets': ['right.kml']},
        ]}
        result = attach_center_tour(
            scene, 'http://42-a:18112/new.kml', 'http://42-a:18112/')
        self.assertEqual(['left.kml'], result['windows'][0]['assets'])
        self.assertEqual(
            ['keep.kml', 'http://42-a:18112/new.kml'],
            result['windows'][1]['assets'])
        self.assertEqual(['right.kml'], result['windows'][2]['assets'])
        self.assertEqual(0, result['duration'])
        self.assertTrue(result['preserve_base'])

    def test_adds_only_center_when_scene_has_no_earth(self):
        result = attach_center_tour(
            {'windows': [{'activity': 'no_activity'}]},
            'http://42-a:18112/new.kml', 'http://42-a:18112/')
        earth = [window for window in result['windows']
                 if window.get('activity') == 'earth']
        self.assertEqual(1, len(earth))
        self.assertEqual('center', earth[0]['presentation_viewport'])

    def test_adds_configured_solo_viewport_when_scene_has_no_earth(self):
        result = attach_center_tour(
            {'windows': [{'activity': 'cesium'}]},
            'http://42-a:18112/new.kml', 'http://42-a:18112/',
            viewport='wall_a')
        earth = [window for window in result['windows']
                 if window.get('activity') == 'earth']
        self.assertEqual(1, len(earth))
        self.assertEqual('wall_a', earth[0]['presentation_viewport'])
        self.assertEqual(
            ['http://42-a:18112/new.kml'], earth[0]['assets'])


if __name__ == '__main__':
    unittest.main()
