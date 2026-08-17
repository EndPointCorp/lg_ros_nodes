#!/usr/bin/env python3
PKG = 'lg_earth'
NAME = 'test_query_queue'

import unittest
import os

from lg_earth.query_queue import QueryQueue

TEST_FILE = '/tmp/query_queue_test'
TEST_QUERY = 'search=asdf'


def consume_query():
    try:
        with open(TEST_FILE, 'r') as f:
            query = f.read()
    except Exception:
        return None
    os.remove(TEST_FILE)
    return query


class TestQueryQueue(unittest.TestCase):
    def setUp(self):
        self.q = QueryQueue(TEST_FILE)

    def tearDown(self):
        self.q.stop()
        self.q.clear()
        consume_query()

    def test_post_query(self):
        self.q.post_query(TEST_QUERY)
        query = consume_query()
        self.assertEquals(TEST_QUERY, query)

    def test_coalesces_only_queries_with_the_same_key(self):
        self.q.post_query('current')
        self.q.post_query('old-camera', coalesce_key='absolute_view')
        self.q.post_query('search=keep-me')
        self.q.post_query('latest-camera', coalesce_key='absolute_view')

        self.assertEqual([
            ('latest-camera', 'absolute_view'),
            ('search=keep-me', None),
        ], list(self.q.q))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestQueryQueue)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
