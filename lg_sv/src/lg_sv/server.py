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

X_THRESHOLD = 0.50
FORWARD_THRESHOLD = .3
BACKWARDS_THRESHOLD = .3
# TODO figure out some good values here
COEFFICIENT_LOW = 0.1
COEFFICIENT_HIGH = 3


def clamp(val, low, high):
    return min(max(val, low), high)


def wrap(val, low, high):
    if val > high:
        val -= (high - low)
    if val < low:
        val += (high - low)
    return val


class StreetviewUtils:
    @staticmethod
    def get_metadata_from_lat_lon(lat, lon, radius=2000):
        """
        Returns a panoid if one exists within $radius meters(?) of the lat/lon
        """
        # this url may change someday...
        url = 'http://maps.google.com/cbk?output=json&v=4&dm=0&pm=0&ll={},{}&radius={}'
        r = requests.get(url.format(lat, lon, radius))
        if r.status_code != 200:
            return False
        content = {}
        try:
            content = json.loads(r.content)
        except ValueError:
            return False
        return content

    @staticmethod
    def get_panoid_from_lat_lon(lat, lon, radius=2000):
        content = StreetviewUtils.get_metadata_from_lat_lon(lat, lon, radius)
        try:
            assert content['Location']
            assert content['Location']['panoId']
        except AssertionError:
            return False
        return str(content['Location']['panoId'])

    @staticmethod
    def translate_server_metadata_to_client_form(metadata):
        """
        The metadata we get from the webapp client looks different from
        the metadata we get directly from google, we should use the webapp
        style as that is where our metadata will come from most of the time

        This is a stripped down metadata with just the essiential bits of
        information in it
        """
        assert isinstance(metadata, dict)
        links = []
        ret = {}
        try:
            for link in metadata['Links']:
                links.append(
                    {
                        'heading': link['yawDeg'],
                        'pano': link['panoId']
                    }
                )
            ret = {
                'links': links,
                'location': {
                    'latLng': {
                        'lat': metadata['Location']['lat'],
                        'lng': metadata['Location']['lng']
                    },
                    'pano': metadata['Location']['panoId']
                }
            }
        except KeyError:
            return {}
        return ret


class PanoViewerServer:
    def __init__(self, location_pub, panoid_pub, pov_pub, tilt_min, tilt_max,
                 nav_sensitivity, space_nav_interval):
        self.location_pub = location_pub
        self.panoid_pub = panoid_pub
        self.pov_pub = pov_pub

        self.last_metadata = dict()
        self.location = Pose2D()
        self.pov = Quaternion()
        self.panoid = str()
        self.state = True
        ### parameterize
        self.nav_sensitivity = nav_sensitivity
        self.tilt_max = tilt_max
        self.tilt_min = tilt_min
        self.space_nav_interval = space_nav_interval
        self.move_forward = 0
        self.move_backward = 0
        self.nearby_panos = NearbyPanos()
        self.last_nav_msg_t = 0
        self.time_since_last_nav_msg = 0

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
        self.nearby_panos.handle_metadata_msg(metadata)

    def get_metadata(self):
        """
        Get the metadata from nearby panos
        """
        return self.nearby_panos.get_metadata()

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
        self.panoid_pub.publish(panoid)

    def handle_panoid_msg(self, panoid):
        """
        Grabs the new panoid from a publisher
        """
        self.panoid = panoid.data
        self.nearby_panos.set_panoid(self.panoid)

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

        # On the first ever nav msg, just set last nav time
        if self.last_nav_msg_t == 0:
            self.last_nav_msg_t = rospy.get_time()
            return

        now = rospy.get_time()
        self.time_since_last_nav_msg = now - self.last_nav_msg_t
        self.last_nav_msg_t = now
        coefficient = self.getCoefficient()
        # attempt deep copy
        pov_msg = Quaternion(self.pov.x, self.pov.y, self.pov.z, self.pov.w)
        tilt = pov_msg.x - coefficient * twist.angular.y * self.nav_sensitivity
        heading = pov_msg.z - coefficient * twist.angular.z * self.nav_sensitivity
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
            if (self.move_forward == 0 or
                    self.time_since_last_nav_msg +
                    self.move_forward < FORWARD_THRESHOLD):
                self.move_forward += self.time_since_last_nav_msg
            if self.time_since_last_nav_msg + self.move_forward > FORWARD_THRESHOLD:
                self._move_forward()
        elif twist.linear.x < -X_THRESHOLD:
            if (self.move_backward == 0 or
                    self.time_since_last_nav_msg +
                    self.move_backward < BACKWARDS_THRESHOLD):
                self.move_backward += self.time_since_last_nav_msg
            if self.time_since_last_nav_msg + self.move_backward > BACKWARDS_THRESHOLD:
                self._move_backward()
        else:
            # reset counters
            self.move_forward = 0
            self.move_backward = 0

    def _move_forward(self):
        """
        Wrapper around move function, resets counter
        """
        if self.move(self.pov.z):
            self.move_forward = 0
            self.move_backward = 0

    def _move_backward(self):
        """
        Wrapper around move function, resets counter and passes an adjusted
        heading
        """
        if self.move((self.pov.z + 180) % 360):
            self.move_backward = 0
            self.move_forward = 0

    def move(self, heading):
        """
        Moves to the closest pano in the direction of the heading
        """
        move_to = self.nearby_panos.find_closest(self.panoid, heading)
        if not move_to:
            return None  # don't update anything
        self.pub_panoid(move_to)
        return True

    def getCoefficient(self):
        """
        Find the ratio of time in between nav messages and
        expected interval. Clamp the result and return.
        """
        coefficient = self.time_since_last_nav_msg / self.space_nav_interval
        coefficient = clamp(coefficient, COEFFICIENT_LOW, COEFFICIENT_HIGH)
        return coefficient


class NearbyPanos:
    def __init__(self):
        self.panoid = None
        self.metadata = None

    def handle_metadata_msg(self, metadata):
        tmp = None
        try:
            tmp = json.loads(metadata.data)
            if tmp['location']['pano'] == self.panoid or self.panoid is None:
                self.metadata = tmp
        except ValueError:
            pass
        except KeyError:
            pass

    def set_panoid(self, panoid):
        self.panoid = panoid

    def find_closest(self, panoid, pov_z):
        """
        Returns the pano that is closest to the direction pressed on the
        spacenav (either forwards or backwards) based on the nearby panos
        bearing to the current pano
        """
        if not self.get_metadata():
            return None
        if 'links' not in self.metadata or not isinstance(self.metadata['links'], list):
            return None
        self.panoid = panoid
        my_lat = self.metadata['location']['latLng']['lat']
        my_lng = self.metadata['location']['latLng']['lng']
        # set closest to the farthest possible result
        closest = 90
        closest_pano = None
        for data in self.metadata['links']:
            tmp = self.headingDifference(pov_z, float(data['heading']))
            if tmp <= closest:
                closest = tmp
                closest_pano = data['pano']
        return closest_pano

    def headingDifference(self, source, target):
        """
        Finds the difference between two headings, takes into account that
        the value 359 degrees is closer to 0 degrees than 10 degrees is
        """
        diff = abs(target - source) % 360
        return diff if diff < 180 else diff - (diff - 180)

    def get_metadata(self):
        """
        Only return the metadata if it matches the current panoid
        """
        if not self.panoid:
            return None
        if not self.metadata:
            return None
        if 'location' not in self.metadata or 'pano' not in self.metadata['location']:
            return None
        if self.metadata['location']['pano'] != self.panoid:
            return None
        return self.metadata
