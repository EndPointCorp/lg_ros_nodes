#!/usr/bin/env python
PKG = 'lg_earth'
NAME = 'test_query_writer'

import unittest
import os

from lg_earth import QueryWriter
from std_msgs.msg import String
from geometry_msgs.msg import Pose

TEST_FILE = '/tmp/query_writer_test'
TOUR_NAME = 'Amazonian Village'
EXIT_TOUR = ''
PLANET_NAME = 'mars'
SEARCH_QUERY = '10010'
TEST_POSE_LON = 20
TEST_POSE_LAT = 31.5
TEST_POSE_ALT = 22
TEST_POSE_HEADING = 270
TEST_POSE_TILT = -12
TEST_POSE_ROLL = 0
TEST_POSE_RANGE = 5000
FLYTO_KML = ('<Camera><latitude>{}</latitude><longitude>{}</longitude>'
             '<altitude>{}</altitude><heading>{}</heading><tilt>{}</tilt>'
             '<roll>{}</roll><altitudeMode>absolute</altitudeMode></Camera>').format(
    TEST_POSE_LAT,
    TEST_POSE_LON,
    TEST_POSE_ALT,
    TEST_POSE_HEADING,
    TEST_POSE_TILT,
    TEST_POSE_ROLL)


class TestQueryWriter(unittest.TestCase):
    def setUp(self):
        self.writer = QueryWriter(TEST_FILE)

    def tearDown(self):
        try:
            os.remove(TEST_FILE)
        except:
            pass

    def test_handle_playtour(self):
        tour = String(TOUR_NAME)
        self.writer.handle_tour(tour)

        expected = 'playtour={}'.format(TOUR_NAME)

        with open(TEST_FILE, 'r') as f:
            content = f.read()
        self.assertEqual(content, expected)

    def test_handle_exittour(self):
        tour = String(EXIT_TOUR)
        self.writer.handle_tour(tour)

        expected = 'exittour=true'

        with open(TEST_FILE, 'r') as f:
            content = f.read()
        self.assertEqual(content, expected)

    def test_handle_planet(self):
        planet = String(PLANET_NAME)
        self.writer.handle_planet(planet)

        expected = 'planet={}'.format(PLANET_NAME)

        with open(TEST_FILE, 'r') as f:
            content = f.read()
        self.assertEqual(content, expected)

    def test_handle_search(self):
        search = String(SEARCH_QUERY)
        self.writer.handle_search(search)

        expected = 'search={}'.format(SEARCH_QUERY)

        with open(TEST_FILE, 'r') as f:
            content = f.read()
        self.assertEqual(content, expected)

    def test_flyto_kml(self):
        kml = String(FLYTO_KML)
        self.writer.handle_flyto_kml(kml)
        print FLYTO_KML
        expected = 'flytoview={}'.format(FLYTO_KML)

        with open(TEST_FILE, 'r') as f:
            content = f.read()
        print content
        self.assertEqual(content, expected)

    def test_flyto_pose_camera(self):
        pose = Pose()
        pose.position.x = TEST_POSE_LON
        pose.position.y = TEST_POSE_LAT
        pose.position.z = TEST_POSE_ALT
        pose.orientation.z = TEST_POSE_HEADING
        pose.orientation.x = TEST_POSE_TILT
        pose.orientation.y = TEST_POSE_ROLL

        self.writer.handle_flyto_pose_camera(pose)

        with open(TEST_FILE, 'r') as f:
            content = f.read()

        parts = content.split('=')
        self.assertEqual(2, len(parts))
        self.assertEqual('flytoview', parts[0])
        kml = parts[1]
        expected_kml = ('<Camera><latitude>{}</latitude>'
                        '<longitude>{}</longitude><altitude>{}</altitude>'
                        '<heading>{}</heading><tilt>{}</tilt><roll>{}</roll>'
                        '<altitudeMode>absolute</altitudeMode></Camera>').format(
            TEST_POSE_LAT,
            TEST_POSE_LON,
            TEST_POSE_ALT,
            TEST_POSE_HEADING,
            TEST_POSE_TILT,
            TEST_POSE_ROLL)
        self.assertEqual(expected_kml, kml)

    def test_flyto_pose_lookat(self):
        pose = Pose()
        pose.position.x = TEST_POSE_LON
        pose.position.y = TEST_POSE_LAT
        pose.position.z = TEST_POSE_ALT
        pose.orientation.z = TEST_POSE_HEADING
        pose.orientation.x = TEST_POSE_TILT
        pose.orientation.y = TEST_POSE_RANGE

        self.writer.handle_flyto_pose_lookat(pose)

        with open(TEST_FILE, 'r') as f:
            content = f.read()

        parts = content.split('=')
        self.assertEqual(2, len(parts))
        self.assertEqual('flytoview', parts[0])
        kml = parts[1]
        expected_kml = ('<LookAt><latitude>{}</latitude>'
                        '<longitude>{}</longitude><altitude>{}</altitude>'
                        '<heading>{}</heading><tilt>{}</tilt><range>{}</range>'
                        '<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>').format(
            TEST_POSE_LAT,
            TEST_POSE_LON,
            TEST_POSE_ALT,
            TEST_POSE_HEADING,
            TEST_POSE_TILT,
            TEST_POSE_RANGE)
        self.assertEqual(expected_kml, kml)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestQueryWriter)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
