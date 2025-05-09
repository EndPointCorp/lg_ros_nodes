#!/usr/bin/env python3

import rospy
import json

from geometry_msgs.msg import Pose2D, Quaternion, Twist
from lg_common.helpers import get_activity_config_or_asset, on_new_scene, make_soft_relaunch_callback, get_first_activity_from_scene, has_activity, handle_initial_state
from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.msg import ApplicationState
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from math import atan2, cos, sin, pi
from lg_sv import PanoViewerServer, NearbyPanos, NearbyStreetviewPanos
from lg_common.helpers import run_with_influx_exception_handler
from lg_msg_defs.srv import PanoIdState


# spacenav_node -> mux_twists -> /lg_twister/twist -> handle_spacenav_msg:
# 1. change pov based on rotational axes -> /<server_type>/pov
# 2. check for movement -> /<server_type>/panoid

# /<server_type>/location -> handle_location_msg:
# 1. query api, publish -> /<server_type>/panoid
# low priority

# /<server_type>/metadata -> handle_metadata_msg:
# 1. update self.metadata


DEFAULT_TILT_MIN = -80
DEFAULT_TILT_MAX = 80
DEFAULT_ZOOM_MIN = 10
DEFAULT_ZOOM_MAX = 30
DEFAULT_NAV_SENSITIVITY = 1.0
DEFAULT_NAV_INTERVAL = 0.02
DEFAULT_TICK_RATE = 180
DEFAULT_IDLE_TIME_UNTIL_SNAP = 1.25
X_THRESHOLD = 0.50
NODE_NAME = 'pano_viewer_server'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


def main():
    rospy.init_node('pano_viewer_server', anonymous=True)
    server_type = rospy.get_param('~server_type', 'streetview')
    location_pub = rospy.Publisher('/%s/location' % server_type,
                                   Pose2D, queue_size=1)
    panoid_pub = rospy.Publisher('/%s/panoid' % server_type,
                                 String, queue_size=1)
    pov_pub = rospy.Publisher('/%s/pov' % server_type,
                              Quaternion, queue_size=2)
    metadata_pub = rospy.Publisher('/%s/metadata' % server_type,
                                   String, queue_size=10)
    director_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=1)

    tilt_min = rospy.get_param('~tilt_min', DEFAULT_TILT_MIN)
    tilt_max = rospy.get_param('~tilt_max', DEFAULT_TILT_MAX)
    zoom_min = rospy.get_param('~zoom_min', DEFAULT_ZOOM_MIN)
    zoom_max = rospy.get_param('~zoom_max', DEFAULT_ZOOM_MAX)
    nav_sensitivity = rospy.get_param('~nav_sensitivity', DEFAULT_NAV_SENSITIVITY)
    x_threshold = rospy.get_param('~x_threshold', X_THRESHOLD)
    space_nav_interval = rospy.get_param('~space_nav_interval', DEFAULT_NAV_INTERVAL)
    nearby_class = rospy.get_param('~nearby_class', 'NearbyStreetviewPanos')
    nearby = get_nearby(nearby_class)
    inverted = str(rospy.get_param('~inverted', "false")).lower() == "true"
    nearby.invert(inverted)
    tick_rate = rospy.get_param('~tick_rate', DEFAULT_TICK_RATE)
    idle_time_until_snap = rospy.get_param('~idle_time_until_snap', DEFAULT_IDLE_TIME_UNTIL_SNAP)

    server = PanoViewerServer(location_pub, panoid_pub, pov_pub, tilt_min, tilt_max,
                              nav_sensitivity, space_nav_interval, idle_time_until_snap, x_threshold,
                              nearby, metadata_pub, zoom_max, zoom_min, tick_rate, director_pub=director_pub,
                              server_type=server_type)

    visibility_publisher = rospy.Publisher('/%s/state' % server_type, ApplicationState, queue_size=1)

    rospy.Subscriber('/%s/location' % server_type, Pose2D,
                     server.handle_location_msg)
    rospy.Subscriber('/%s/metadata' % server_type, String,
                     server.handle_metadata_msg)
    rospy.Subscriber('/%s/panoid' % server_type, String,
                     server.handle_panoid_msg)
    rospy.Subscriber('/%s/pov' % server_type, Quaternion,
                     server.handle_pov_msg)
    rospy.Subscriber('/lg_twister/twist', Twist,
                     server.handle_spacenav_msg)
    rospy.Subscriber('/%s/state' % server_type, ApplicationState,
                     server.handle_state_msg)
    rospy.Subscriber('/%s/raw_metadata' % server_type, String,
                     server.handle_raw_metadata_msg)
    rospy.Subscriber('/spacenav/joy', Joy, server.handle_joy)
    rospy.Subscriber('/%s/tilt_snappy' % server_type, Bool, server.handle_tilt_snappy)
    rospy.Service('/%s/panoid_state' % server_type, PanoIdState, server.get_panoid)
    make_soft_relaunch_callback(server.handle_soft_relaunch, groups=['streetview'])

    # This will translate director messages into /<server_type>/panoid messages
    def handle_director_message(scene):
        logger.debug('running handle director w/ scene: %s' % scene)
        _server_type = server_type

        has_asset = has_activity(scene, _server_type)
        has_no_activity = has_activity(scene, 'no_activity')
        if has_no_activity:
            logger.debug('ignoring scene due to no_activity')
            return
        if not has_asset:
            logger.debug('hiding self')
            visibility_publisher.publish(ApplicationState(state='HIDDEN'))
            return
        if scene.get('slug', '') == 'auto_generated_sv_scene':
            logger.debug("Ignoring my own generated message")
            asset = get_activity_config_or_asset(scene, _server_type)
            panoid = asset.get('panoid', '')
            logger.debug("length of panoid is %s server type %s" % (len(panoid), server_type))
            logger.debug("panoid is %s" % panoid)
            #TODO Need to remove this block, once the nonfree streetview viewer is fixed
            if server_type == 'streetview':
                visibility_publisher.publish(ApplicationState(state='VISIBLE'))
                logger.debug("publishing visible for {}".format(server_type))
                return
            elif server_type == 'panoviewer' or server_type == 'panovideo':
                visibility_publisher.publish(ApplicationState(state='VISIBLE'))
                return
            visibility_publisher.publish(ApplicationState(state='HIDDEN'))
            return

        if server_type == 'streetview':
            asset = get_activity_config_or_asset(scene, server_type)
            panoid = asset.get('panoid', '')
        else:
            panoid = scene['windows'][0]['assets'][0]

        visibility_publisher.publish(ApplicationState(state='VISIBLE'))

        pov = server.pov
        try:
            pov.x = float(asset['tilt'])
        except Exception:
            pov.x = 0
        try:
            pov.z = float(asset['heading'])
            # we don't want to flip the auto generated ones, or the non_inverted
            if scene.get('slug', '') != 'auto_generated_sv_scene' and inverted:
                pov.z = (pov.z + 180) % 360
        except Exception:
            pov.z = 0
        pov.w = zoom_max
        server.pub_panoid(panoid, pov=pov, no_director=True)

    def initial_state_handler(uscs_msg):
        try:
            logger.debug("about to load json: %s" % uscs_msg.message)
            scene = json.loads(uscs_msg.message)
        except Exception:
            return
        handle_director_message(scene)

    on_new_scene(handle_director_message)
    handle_initial_state(initial_state_handler)

    rospy.spin()


def get_nearby(n):
    if n == 'NearbyStreetviewPanos':
        return NearbyStreetviewPanos()
    if n == 'NearbyPanos':
        return NearbyPanos()
    return NearbyPanos()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
