#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, Quaternion, Twist
from std_msgs.msg import String
from math import atan2, cos, sin, pi


# spacenav_node -> /spacenav/twist -> handle_spacenav_msg:
# 1. change pov based on rotational axes -> /streetview/pov
# 2. check for movement -> /streetview/panoid

# /streetview/location -> handle_location_msg:
# 1. query api, publish -> /streetview/panoid
# low priority

# /streetview/metadata -> handle_metadata_msg:
# 1. update self.metadata


DEFAULT_TILT_MIN = -80
DEFAULT_TILT_MAX = 80
DEFAULT_NAV_SENSITIVITY = 1.0


def clamp(val, low, high):
    return min(max(val, low), high)


def wrap(val, low, high):
    if val > high:
        val -= (high - low)
    if val < low:
        val += (high - low)
    return val

def bearing(lat1d, lon1d, lat2d, lon2d):
	lat1 = lat1d * (pi / 180)
	lat2 = lat2d * (pi / 180)
	lon1 = lon1d * (pi / 180)
	lat2 = lat2d * (pi / 180)
	bearing_r = atan2(sin(lon2 - lon1) * cos(lat2),
				 cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1))
	return bearing_r * 180 / pi

def headingDifference(source, target):
	diff = abs(target - source) % 360
	return diff if diff < 180 else diff - (diff - 180)


class StreetviewServer:
    def __init__(self, location_pub, panoid_pub, pov_pub, tilt_min, tilt_max, nav_sensitivty):
        self.location_pub = location_pub
        self.panoid_pub = panoid_pub
        self.pov_pub = pov_pub

        self.last_metadata = dict()
        self.location = Pose2D()
        self.pov = Quaternion()
        self.panoid = str()
        self.metadata = dict()
        ### parameterize
        self.nav_sensitivity = nav_sensitivity
        self.tilt_max = tilt_max
        self.tilt_min = tilt_min

    def pub_location(self, pose2d):
        self.location_pub(pose2d)

    def handle_location_msg(self, pose2d):
        self.location = pose2d

    def handle_metadata_msg(self, metadata):
        self.last_metadata = metadata

    def handle_pov_msg(self, quaternion):
        self.pov = quaternion

    def pub_panoid(self, panoid):
        self.panoid_pub.publish(panoid)

    def handle_panoid_msg(self, panoid):
        self.panoid = panoid

    def handle_spacenav_msg(self, twist):
        # attempt deep copy
        pov_msg = Quaternion(self.pov.x, self.pov.y, self.pov.z, self.pov.z)
        # or maybe Quaternion(self.pov.x, self.pov.y, ...)
        tilt = pov_msg.x + twist.angular.x * self.nav_sensitivity
        heading = pov_msg.z + twist.angular.z * self.nav_sensitivity
        pov_msg.x = clamp(tilt, self.tilt_min, self.tilt_max)
        pov_msg.z = wrap(heading, 0, 360)
        self.pov_pub.publish(pov_msg)

	def get_nearby_panos(self, panoid):
		# this url may change someday...
		url = 'http://maps.google.com/cbk?output=json&v=4&dm=0&pm=0&panoid={}'
		r = requests.get(url.format(panoid))
		if r.status_code != 200:
			return False #maybe raise exception?
		content = {}
		try:
			content = json.loads(r.content)
			assert content['Links']
		except ValueError:
			return False
		except KeyError:
			return False
		links = []
		for link in content['Links']:
			links.append(str(link['panoId']))
		return links

	def get_panoid_from_lat_lon(self, lat, lon, radius=2000):
		# this url may change someday...
		url = 'http://maps.google.com/cbk??output=json&v=4&dm=0&pm=0&ll={},{}&radius={}'
		r = requests.get(url.format(lat, lon, radius))
		if r.status_code != 200:
			return False
		content = {}
		try:
			content = json.loads(r.content)
			assert content['Location']
			assert content['Location']['panoId']
		except ValueError:
			return False
		except KeyError:
			return False
		return str(content['Location']['panoId'])


if __name__ == '__main__':
    rospy.init_node('streetview_server')
    location_pub = rospy.Publisher('/streetview/location',
                              Pose2D, queue_size=1)
    panoid_pub = rospy.Publisher('/streetview/panoid',
                              String, queue_size=1)
    pov_pub = rospy.Publisher('/streetview/pov',
                              Quaternion, queue_size=2)

    tilt_min = rospy.get_param('~tilt_min', DEFAULT_TILT_MIN)
    tilt_max = rospy.get_param('~tilt_max', DEFAULT_TILT_MAX)
    nav_sensitivity = rospy.get_param('~nav_sensitivity', DEFAULT_NAV_SENSITIVITY)

    server = StreetviewServer(location_pub, panoid_pub, pov_pub, tilt_min, tilt_max, nav_sensitivity)

    rospy.Subscriber('/streetview/location', Pose2D,
                     server.handle_location_msg)
    rospy.Subscriber('/streetview/metadata', String,
                     server.handle_metadata_msg)
    rospy.Subscriber('/streetview/panoid', String,
                     server.handle_panoid_msg)
    rospy.Subscriber('/streetview/pov', Quaternion,
                     server.handle_pov_msg)
    rospy.Subscriber('/spacenav/twist', Twist,
                     server.handle_spacenav_msg)
    rospy.spin()
"""
import unittest

TEST_X = 1
TEST_Y = 2
TEST_Z = 3
TEST_W = 4.2

TEST_TWIST_X = 1
TEST_TWIST_Y = 1
TEST_TWIST_Z = 1

TEST_PANOID = "ZphZajKeR_lJjBJftY0SYw"

class TestStreetviewServer(unittest.TestCase):
    def setUp(self):
        self.server = StreetviewServer()

    def test_pov_msg(self):
        msg = Quaternion(
            x=TEST_X, y=TEST_Y, z=TEST_Z, w=TEST_W
        )
        self.server.handle_pov_msg(msg)
        self.assertEqual(TEST_X, self.server.pov.x)
        self.assertEqual(TEST_Y, self.server.pov.y)
        self.assertEqual(TEST_Z, self.server.pov.z)
        self.assertEqual(TEST_W, self.server.pov.w)

    # test handle_spacenav_msg with mock publisher
    def test_spacenav_msg(self):
        msg = Twist()
        msg.angular.x = TEST_TWIST_X
        msg.angular.y = TEST_TWIST_Y
        msg.angular.z = TEST_TWIST_Z
        initial_pov = Quaternion(
            x=0, y=0, z=0, w=0
        )
        self.server.handle_pov_msg(initial_pov)

        self.server.handle_spacenav_msg(msg)
        self.assertEqual(TEST_TWIST_X*self.server.nav_sensitivity,
                         self.server.pov.x)
        self.assertEqual(TEST_TWIST_Y*self.server.nav_sensitivity,
                         self.server.pov.y)
        self.assertEqual(TEST_TWIST_Z*self.server.nav_sensitivity,
                         self.server.pov.z)

     def test_spacenav_tilt_max(self):
         msg = Twist()
         msg.angular.x = TEST_TWIST_X
         msg.angular.y = 0
         msg.angular.z = 0
         initial_pov = Quaternion(
           x=self.server.tilt_max, y=0, z=0, w=0
         )
         self.server.handle_pov_msg(initial_pov)
         self.server.handle_spacenav(msg)
         self.assertEqual(self.server.tilt_max, self.server.pov.x)


     def test_spacenav_tilt_min(self):
         msg = Twist()
         msg.angular.x = 0 - TEST_TWIST_X
         msg.angular.y = 0
         msg.angular.z = 0
         tilt_min = 0 - self.server.tilt_max
         initial_pov = Quaternion(
           x=tilt_min, y=0, z=0, w=0
         )
         self.server.handle_pov_msg(initial_pov)
         self.server.handle_spacenav(msg)
         self.assertEqual(tilt_min, self.server.pov.x)

     def test_panoid_msg(self):
         panoid = TEST_PANOID
         self.server.handle_panoid(panoid)
         self.assertEqual(TEST_PANOID,self.server.panoid)
"""
