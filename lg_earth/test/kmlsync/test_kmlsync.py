#!/usr/bin/env python

PKG = 'lg_earth'
NAME = 'test_kmlsync'
KMLSYNC_HOST = '127.0.0.1'
KMLSYNC_PORT = 8765
KML_ENDPOINT = 'http://' + KMLSYNC_HOST + ':' + str(KMLSYNC_PORT)

import unittest
import requests
import rospy

from lg_earth import KMLSyncServer

# - start webserver and pretend you are google earth client
# - start webserver and send director message and pretend you are google earth client

class TestKMLSync(unittest.TestCase):
    def setUp(self):
        rospy.loginfo("Starting test for KMLSync")

    def test_master_kml(self):
        r = requests.get(KML_ENDPOINT + '/master.kml')
        self.assertEqual(r.status_code, 200)

    def test_network_link_update_kml(self):
        r = requests.get(KML_ENDPOINT + '/network_link_update.kml')
        self.assertEqual(r.status_code, 200)

    def test_network_link_update_cookie_string_is_initially_empty(self):
        r = requests.get(KML_ENDPOINT + '/network_link_update.kml')
        pass



if __name__ == '__main__':
    import rostest
    rospy.sleep(1)
    rostest.rosrun(PKG, NAME, TestKMLSync)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
