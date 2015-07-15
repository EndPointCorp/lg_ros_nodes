import rospy
from geometry_msgs.msg import Pose2D, Quaternion, Twist
from lg_common.msg import ApplicationState
from math import atan2, cos, sin, pi
import requests
import json


# spacenav_node -> /spacenav/twist -> handle_spacenav_msg:
# 1. change pov based on rotational axes -> /streetview/pov
# 2. check for movement -> /streetview/panoid

# /streetview/location -> handle_location_msg:
# 1. query api, publish -> /streetview/panoid
# low priority

# /streetview/metadata -> handle_metadata_msg:
# 1. update self.metadata

X_THRESHOLD = 0.67
FORWARD_THRESHOLD = 10
BACKWARD_THRESHOLD = 10

def clamp(val, low, high):
    return min(max(val, low), high)


def wrap(val, low, high):
    if val > high:
        val -= (high - low)
    if val < low:
        val += (high - low)
    return val

class StreetviewUtils:
    def get_panoid_from_lat_lon(lat, lon, radius=2000):
        """
        Returns a panoid if one exists within $radius meters(?) of the lat/lon
        """
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


class StreetviewServer:
    def __init__(self, location_pub, panoid_pub, pov_pub, tilt_min, tilt_max, nav_sensitivity):
        self.location_pub = location_pub
        self.panoid_pub = panoid_pub
        self.pov_pub = pov_pub

        self.last_metadata = dict()
        self.location = Pose2D()
        self.pov = Quaternion()
        self.panoid = str()
        self.metadata = dict()
        self.state = True
        ### parameterize
        self.nav_sensitivity = nav_sensitivity
        self.tilt_max = tilt_max
        self.tilt_min = tilt_min
        self.move_forward = 0
        self.move_backward = 0
        self.nearby_panos = NearbyPanos()

    def pub_location(self, pose2d):
        """
        Publishes new location after setting the instance variable
        """
        self.location = pose2d
        self.location_pub.publish(pose2d)

    def handle_location_msg(self, pose2d):
        """
        Grabs the new position, and finds the corresponding panoid
        then publishes the new panoid
        """
        self.location = pose2d
        panoid = StreetviewUtils.get_panoid_from_lat_lon(self.location.x, self.location.y)
        if panoid:
            self.pub_panoid(panoid)

    def handle_metadata_msg(self, metadata):
        """
        Grabs the new metadata from a publisher
        """
        self.last_metadata = metadata

    def pub_pov(self, pov):
        """
        Publishes the new pov after setting the instance variable
        """
        self.pov = pov
        self.pov_pub.publish(pov)

    def handle_pov_msg(self, quaternion):
        """
        Grabs the new pov from a publisher
        """
        self.pov = quaternion

    def pub_panoid(self, panoid):
        """
        Publishes a new panoid after setting the instance variable
        """
        self.panoid = panoid
        self.panoid_pub.publish(panoid)

    def handle_panoid_msg(self, panoid):
        """
        Grabs the new panoid from a publisher
        """
        self.panoid = panoid

    def handle_state_msg(self, app_state):
        """
        Set state to true if the application is visible
        """
        self.state = (app_state.state == ApplicationState.VISIBLE)

    def handle_spacenav_msg(self, twist):
        """
        Adjust pov based on the twist message received, also handle
        a possible change of pano
        """
        if not self.state:
            return
        # attempt deep copy
        pov_msg = Quaternion(self.pov.x, self.pov.y, self.pov.z, self.pov.w)
        # or maybe Quaternion(self.pov.x, self.pov.y, ...)
        tilt = pov_msg.x - twist.angular.y * self.nav_sensitivity
        heading = pov_msg.z - twist.angular.z * self.nav_sensitivity
        pov_msg.x = clamp(tilt, self.tilt_min, self.tilt_max)
        pov_msg.z = wrap(heading, 0, 360)
        self.pub_pov(pov_msg)
        # check to see if the pano should be moved
        self.handle_possible_pano_change(twist)

    def handle_possible_pano_change(self, twist):
        """
        Only moves if the linear x is > or < the x_threshold and that it has
        been that way for atleast {backward,forward}_threshold publications
        """
        if twist.linear.x > X_THRESHOLD:
            self.move_forward += 1
            if self.move_forward > FORWARD_THRESHOLD:
                self.move_forward()
        elif twist.linear.x < -X_THRESHOLD:
            self.move_backward += 1
            if self.move_backward > BACKWARD_THRESHOLD:
                self.move_backward()
        else:
            # reset counters
            self.move_forward = 0
            self.move_backward = 0

    def move_forward(self):
        """
        Wrapper around move function, resets counter
        """
        self.move_forward = 0
        self.move(self.pov.z)

    def move_backward(self):
        """
        Wrapper around move function, resets counter and passes an adjusted
        heading
        """
        self.move_backward = 0
        self.move((self.pov.z + 180) % 360)

    def move(self, heading):
        """
        Moves to the closest pano in the direction of the heading
        """
        move_to = self.nearby_panos.find_closest(self.panoid, heading)
        self.pub_panoid(move_to)

class NearbyPanos:
    def __init__(self):
        self.panoid = None

    def find_closest(self, panoid, pov_z):
        """
        Returns the pano that is closest to the direction pressed on the
        spacenav (either forwards or backwards) based on the nearby panos
        bearing to the current pano
        """
        self.panoid = panoid
        our_metadata = self.get_pano_metadata(self.panoid)
        my_lat = float(our_metadata['Location']['lat'])
        my_lng = float(our_metadata['Location']['lng'])
        nearby = self.get_nearby_panos()
        nearby_metadata = map(self.get_pano_metadata, nearby)
        # set closest to the farthest possible result
        closest = 180
        closest_pano = ''
        for data in nearby_metadata:
            bearing = self.bearing(
                    my_lat, my_lng, float(data['Location']['lat']),
                    float(data['Location']['lng']))
            tmp = self.headingDifference(pov_z, bearing)
            if tmp <= closest:
                closest = tmp
                closest_pano = data['Location']['panoId']
        return closest_pano

    def get_pano_metadata(self, panoid):
        """
        Returns a pano's metadata, uses an undocumented google api call...
        """
        # this url may change someday...
        url = 'http://maps.google.com/cbk?output=json&v=4&dm=0&pm=0&panoid={}'
        r = requests.get(url.format(panoid))
        if r.status_code != 200:
            return False #maybe raise exception?
        content = {}
        try:
            content = json.loads(r.content)
        except ValueError:
            return False
        return content

    def get_nearby_panos(self, panoid=None):
        """
        Returns an array of nearby panos

        nearby panos are found in the pano metadata['Links'] section
        """
        if not panoid:
            panoid = self.panoid

        content = self.get_pano_metadata(panoid)
        links = []
        # make sure the Links key exists
        if not 'Links' in content:
            return []
        for link in content['Links']:
            links.append(str(link['panoId']))
        return links

    def bearing(self, lat1d, lon1d, lat2d, lon2d):
        """
        Returns the bearing from {lat,lon}1 to {lat,lon}2

        arguments given in degrees
        returns degrees
        """
        lat1 = lat1d * (pi / 180)
        lat2 = lat2d * (pi / 180)
        lon1 = lon1d * (pi / 180)
        lon2 = lon2d * (pi / 180)
        bearing_r = atan2(sin(lon2 - lon1) * cos(lat2),
                    cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1))
        return bearing_r * 180 / pi

    def headingDifference(self, source, target):
        """
        Finds the difference between two headings, takes into account that
        the value 359 degrees is closer to 0 degrees than 10 degrees is
        """
        diff = abs(target - source) % 360
        return diff if diff < 180 else diff - (diff - 180)
