import rospy
from geometry_msgs.msg import Pose2D, Quaternion, Twist
from std_msgs.msg import String
from lg_msg_defs.msg import ApplicationState
from rospy import ROSException
from lg_sv import NearbyPanos
import requests
import json

from interactivespaces_msgs.msg import GenericMessage

from lg_common.logger import get_logger
logger = get_logger('sv_server_class')
# spacenav_node -> mux_twists -> /lg_twister/twist -> handle_spacenav_msg:
# 1. change pov based on rotational axes -> /streetview/pov
# 2. check for movement -> /streetview/panoid

# /streetview/location -> handle_location_msg:
# 1. query api, publish -> /streetview/panoid
# low priority

# /streetview/metadata -> handle_metadata_msg:
# 1. update self.metadata

X_THRESHOLD = 0.50
FORWARD_THRESHOLD = 0.2
BACKWARDS_THRESHOLD = 0.2
MOVEMENT_REPEAT_DELAY = 0.6
# TODO figure out some good values here
COEFFICIENT_LOW = 0.1
COEFFICIENT_HIGH = 3
ZOOM_MIN = 40
ZOOM_MAX = 40
INITIAL_ZOOM = 40
#IDLE_TIME_UNTIL_SNAP = 1.25
SNAP_DURATION = 15.0


def clamp(val, low, high):
    return min(max(val, low), high)


def wrap(val, low, high):
    if val > high:
        val -= (high - low)
    if val < low:
        val += (high - low)
    return val


def mean(series):
    return sum(series) / float(len(series))


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
                        'heading': float(link.get('yawDeg', 0)),
                        'pano': link.get('panoId', '')
                    }
                )
            ret = {
                'links': links,
                'location': {
                    'latLng': {
                        'lat': float(metadata.get('Location', {}).get('lat', 0)),
                        'lng': float(metadata.get('Location', {}).get('lng', 0))
                    },
                    'description': metadata.get('Location', {}).get('description', ''),
                    'attribution_name': metadata.get('Data', {}).get('attribution_name', ''),
                    'pano': metadata.get('Location', {}).get('panoId', '')
                }
            }
        except KeyError:
            return {}
        return ret


class PanoViewerServer:
    def __init__(self, location_pub, panoid_pub, pov_pub, tilt_min, tilt_max,
                 nav_sensitivity, space_nav_interval, idle_time_until_snap,
                 x_threshold=X_THRESHOLD, nearby_panos=NearbyPanos(),
                 metadata_pub=None, zoom_max=ZOOM_MAX, zoom_min=ZOOM_MIN,
                 tick_rate=180, director_pub=None, server_type=""):
        self.location_pub = location_pub
        self.panoid_pub = panoid_pub
        self.pov_pub = pov_pub
        self.server_type = server_type
        self.director_pub = director_pub

        self.nav_sensitivity = nav_sensitivity
        self.tilt_max = tilt_max
        self.tilt_min = tilt_min
        self.space_nav_interval = space_nav_interval
        self.idle_time_until_snap = idle_time_until_snap
        self.nearby_panos = nearby_panos
        self.x_threshold = x_threshold
        self.metadata_pub = metadata_pub
        self.zoom_max = zoom_max
        self.zoom_min = zoom_min
        self.tick_rate = tick_rate
        self.gutter_val = 0.0005
        self.tick_period = 1.0 / float(self.tick_rate)

        self.state = True

        self.initialize_variables()
        self.start_timer()

    def initialize_variables(self):
        self.button_down = False
        self.last_nav_msg_t = 0
        self.last_nongutter_nav_msg_t = 0
        self.time_since_last_nav_msg = 0
        self.move_forward = 0
        self.move_backward = 0
        self.last_metadata = dict()
        self.location = Pose2D()
        self.pov = Quaternion()
        self.pov.w = INITIAL_ZOOM  # TODO is this alright?
        self.panoid = str()
        self.last_twist_msg = Twist()
        self.tilt_method = self.tilt_snappy

    def _twist_is_in_gutter(self, twist_msg):
        return (
            abs(twist_msg.linear.x) < self.gutter_val and
            abs(twist_msg.linear.y) < self.gutter_val and
            abs(twist_msg.linear.z) < self.gutter_val and
            abs(twist_msg.angular.x) < self.gutter_val and
            abs(twist_msg.angular.y) < self.gutter_val and
            abs(twist_msg.angular.z) < self.gutter_val
        )

    def _tick(self, e):
        if e.last_real is None:
            return

        dt = (e.current_real - e.last_real).to_sec()

        npov = self.project_pov(self.last_twist_msg, dt)
        try:
            self.pub_pov(npov)
        except ROSException as error:
            logger.warning("Could not publish pov during _tick: %s" % error)

    def start_timer(self):
        if not hasattr(self, 'tick_timer') or self.tick_timer is None:
            self.tick_timer = rospy.Timer(
                rospy.Duration.from_sec(self.tick_period),
                self._tick
            )
        else:
            logger.warning('Tried to start_timer() a running PanoViewerServer')

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

    def handle_raw_metadata_msg(self, msg):
        metadata = json.loads(msg.data)
        metadata = StreetviewUtils.translate_server_metadata_to_client_form(metadata)
        if self.metadata_pub:
            self.metadata_pub.publish(String(json.dumps(metadata)))

    def get_metadata(self):
        """
        Get the metadata from nearby panos
        """
        return self.nearby_panos.get_metadata()

    def get_panoid(self, *args, **kwargs):
        """
        Get the current panoid
        """
        return self.panoid

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

    def pub_panoid(self, panoid, pov=None, no_director=False):
        """
        Publishes a new panoid after setting the instance variable
        """
        if not no_director:
            self.generate_director_message(panoid, pov)
        if pov:
            self.pub_pov(pov)
        self.panoid = panoid
        self.nearby_panos.set_panoid(self.panoid)

    def tilt_snappy(self, twist_msg, coefficient):
        now = rospy.get_time()
        idle_t = now - self.last_nongutter_nav_msg_t
        if idle_t < self.idle_time_until_snap:
            return self.tilt_not_snappy(twist_msg, coefficient)

        snap_t = idle_t - self.idle_time_until_snap
        tilt = self.pov.x * max(1 - (snap_t / SNAP_DURATION), 0)
        return tilt

    def tilt_not_snappy(self, twist_msg, coefficient):
        tilt = self.pov.x - coefficient * twist_msg.angular.y * self.nav_sensitivity
        return tilt

    def project_pov(self, twist_msg, dt):
        coefficient = dt / self.tick_period / (1.0 / 60.0 / self.tick_period)
        heading = self.pov.z - coefficient * twist_msg.angular.z * self.nav_sensitivity
        tilt = self.tilt_method(twist_msg, coefficient)
        #zoom = self.pov.w + coefficient * twist_msg.linear.z * self.nav_sensitivity
        zoom = INITIAL_ZOOM
        pov_msg = Quaternion(
            x=clamp(tilt, self.tilt_min, self.tilt_max),
            y=0,
            z=wrap(heading, 0, 360),
            w=clamp(zoom, self.zoom_min, self.zoom_max),
        )
        return pov_msg

    def handle_panoid_msg(self, panoid):
        """
        Grabs the new panoid from a publisher
        """
        # Nothing to do here...
        if self.panoid == panoid.data:
            self.nearby_panos.set_panoid(self.panoid)
            return
        self.generate_director_message(panoid.data)
        self.panoid = panoid.data
        self.nearby_panos.set_panoid(self.panoid)
        # now sets up director message so we can set the state of the system

    def generate_director_message(self, panoid, pov=None, asset={}):
        if panoid == self.panoid:
            return
        msg = GenericMessage()
        msg.type = 'json'
        if pov:
            heading = pov.z
            tilt = pov.x
        else:
            heading = self.pov.z
            tilt = self.pov.x
        activity = {
            "activity": self.server_type,
            "assets": [
                panoid
            ],
            "activity_config": {
                "panoid": panoid,
                "heading": heading,
                "tilt": tilt
            }
        }
        if asset == {}:
            message = {
                "slug": "auto_generated_sv_scene",
                "windows": [activity]
            }
        else:
            # asset was passed, keep all director message parts
            message = asset.copy()
            message['slug'] = 'auto_generated_sv_scene'
            message['windows'] = [activity]
        msg.message = json.dumps(message)
        if self.director_pub:
            self.director_pub.publish(msg)

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
        # ignore spacenav messages when self.state != True
        if not self.state:
            return
        now = rospy.get_time()
        self.time_since_last_nav_msg = now - self.last_nav_msg_t
        self.last_nav_msg_t = now

        if self._twist_is_in_gutter(twist):
            self.last_twist_msg = Twist()
        else:
            self.last_nongutter_nav_msg_t = now
            self.last_twist_msg = twist

        # check to see if the pano should be moved
        self.handle_possible_pano_change(twist)

    def handle_possible_pano_change(self, twist):
        """
        Only moves if the linear x is > or < the x_threshold and that it has
        been that way for atleast {backward,forward}_threshold publications
        """
        if twist.linear.x > self.x_threshold:
            if (self.move_forward == 0 or
                    self.time_since_last_nav_msg +
                    self.move_forward < FORWARD_THRESHOLD):
                self.move_forward += self.time_since_last_nav_msg
            if self.time_since_last_nav_msg + self.move_forward > FORWARD_THRESHOLD:
                self._move_forward()
        elif twist.linear.x < -self.x_threshold:
            if (self.move_backward == 0 or
                    self.time_since_last_nav_msg +
                    self.move_backward < BACKWARDS_THRESHOLD):
                self.move_backward += self.time_since_last_nav_msg
            if self.time_since_last_nav_msg + self.move_backward > BACKWARDS_THRESHOLD:
                self._move_backward()
        else:
            # reset counters
            if self.move_forward < 0:
                self.move_forward += MOVEMENT_REPEAT_DELAY / 10.0
            if self.move_backward < 0:
                self.move_backward += MOVEMENT_REPEAT_DELAY / 10.0
            self.move_forward = min(self.move_forward, 0)
            self.move_backward = min(self.move_backward, 0)

    def handle_joy(self, joy):
        """
        Move forward if the button is down, and wasn't previously down
        """
        if not self.state:
            return
        if 1 not in joy.buttons:
            self.button_down = False
            return

        if self.button_down:
            # button is still down
            return
        self.button_down = True
        if joy.buttons[-1] == 1:
            self._move_forward()
        else:
            self._move_backward()

    def _move_forward(self):
        """
        Wrapper around move function, resets counter
        """
        if self.move(self.pov.z):
            self.move_forward = -MOVEMENT_REPEAT_DELAY
            self.move_backward = -MOVEMENT_REPEAT_DELAY

    def _move_backward(self):
        """
        Wrapper around move function, resets counter and passes an adjusted
        heading
        """
        if self.move((self.pov.z + 180) % 360):
            self.move_backward = -MOVEMENT_REPEAT_DELAY
            self.move_forward = -MOVEMENT_REPEAT_DELAY

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

    def handle_soft_relaunch(self, *args, **kwargs):
        """
        Reinitialize all variables
        """
        logger.debug('handling soft relaunch for streetview')
        self.initialize_variables()
        self.nearby_panos.handle_soft_relaunch()

    def handle_tilt_snappy(self, msg):
        if msg.data:
            self.tilt_method = self.tilt_snappy
        else:
            self.tilt_method = self.tilt_not_snappy
