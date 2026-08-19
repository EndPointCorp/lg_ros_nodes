#!/usr/bin/env python3
PKG = 'lg_earth'
NAME = 'test_query_writer'

import unittest
import os

from lg_earth import QueryWriter
from std_msgs.msg import String

TEST_FILE = '/tmp/query_writer_test'
TOUR_NAME = 'Amazonian Village'
EXIT_TOUR = ''
PLANET_NAME = 'mars'
SEARCH_QUERY = '10010'


def consume_query():
    with open(TEST_FILE, 'r') as f:
        q = f.read()
    os.remove(TEST_FILE)
    return q


class TestQueryWriter(unittest.TestCase):
    def setUp(self):
        self.writer = QueryWriter(TEST_FILE)

    def tearDown(self):
        try:
            os.remove(TEST_FILE)
        except Exception:
            pass

        self.writer.shutdown()

    def test_handle_playtour(self):
        tour = String(TOUR_NAME)
        self.writer.handle_tour(tour)

        expected = 'playtour={}'.format(TOUR_NAME)

        content = consume_query()
        self.assertEqual(content, expected)

    def test_handle_exittour(self):
        tour = String(EXIT_TOUR)
        self.writer.handle_tour(tour)

        expected = 'exittour=true'

        content = consume_query()
        self.assertEqual(content, expected)

    def test_handle_planet(self):
        planet = String(PLANET_NAME)
        self.writer.handle_planet(planet)

        expected = 'planet={}'.format(PLANET_NAME)

        content = consume_query()
        self.assertEqual(content, expected)

    def test_handle_search(self):
        search = String(SEARCH_QUERY)
        self.writer.handle_search(search)

        expected = 'search={}'.format(SEARCH_QUERY)

        content = consume_query()
        self.assertEqual(content, expected)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestQueryWriter)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
