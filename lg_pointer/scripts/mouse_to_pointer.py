#!/usr/bin/env python

from threading import Lock
import math
from evdev import ecodes
import evdev
import os
import urllib2
import subprocess
from tempfile import mktemp

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray
from wiimote.msg import State
from lg_mirror.msg import EvdevEvent, EvdevEvents
from lg_mirror.srv import EvdevDeviceInfo, EvdevDeviceInfoResponse
from lg_common.msg import StringArray
from std_srvs.srv import Empty
from lg_pointer import MegaViewport

NODE_NAME = 'wiimote_to_pointer'
TICK_RATE = 65  # Hz
DEV_WIDTH = 8192
DEV_HEIGHT = 8192
MAX_STALE_COUNT = 30


def clamp(val, lo, hi):
    return min(max(val, lo), hi)


def handle_device_info(req):
    res = EvdevDeviceInfoResponse()
    res.bustype = 3
    res.types = [ecodes.EV_ABS, ecodes.EV_KEY]
    res.abs_codes = [ecodes.ABS_X, ecodes.ABS_Y]
    res.rel_codes = []
    res.key_codes = [ecodes.BTN_TOUCH]
    res.abs_min = [0, 0]
    res.abs_max = [DEV_WIDTH, DEV_HEIGHT]
    return res


def grabCustomUdev(udev_location, rules_dest):
    tmp_rules = mktemp()
    with open(tmp_rules, 'w') as f:
        resp = urllib2.urlopen(udev_location)
        if resp.code != 200:
            rospy.logerr("Could not curl the udev rules... continuing without them")
            return
        f.write(resp.read())
    os.system('sudo mv {} {}'.format(tmp_rules, rules_dest))
    os.system('sudo udevadm control --reload-rules; sudo udevadm trigger')


def main():
    rospy.init_node(NODE_NAME)

    udev_location = rospy.get_param(
        '~udev_location',
        'http://lg-head/lg/external_devices/97-logitech-spotlight.rules')
    grabCustomUdev(udev_location, '/etc/udev/rules.d/97-logitech-spotlight.rules')
    device_id = rospy.get_param('~device_id', 'default')
    device_path = rospy.get_param('~device_path', 'default')
    viewports = [
        v.strip() for v in rospy.get_param('~viewports').split(',')
    ]
    arc_width = float(rospy.get_param('~arc_width', math.pi / 2))

    events_topic = '/lg_mirror/{}/events'.format(device_id)
    kbd_events_topic = '/lg_mirror/{}/kbd_events'.format(device_id)
    routes_topic = '/lg_mirror/{}/active_routes'.format(device_id)
    info_srv = '/lg_mirror/{}/device_info'.format(device_id)

    events_pub = rospy.Publisher(events_topic,
                                 EvdevEvents, queue_size=10)
    kbd_events_pub = rospy.Publisher(kbd_events_topic,
                                 EvdevEvents, queue_size=10)
    routes_pub = rospy.Publisher(routes_topic,
                                 StringArray, queue_size=10)
    feedback_pub = rospy.Publisher('/joy/set_feedback',
                                   JoyFeedbackArray, queue_size=10)
    imu_calibrate = rospy.ServiceProxy('/imu/calibrate', Empty, persistent=True)
    mouse_timeout = int(rospy.get_param('~mouse_timeout', 10))
    sleep_time = 0.01  # ooo magic, pretty

    mvp = MegaViewport(viewports, arc_width)

    rospy.Service(info_srv, EvdevDeviceInfo, handle_device_info)

    while not os.path.exists(device_path):
        rospy.logwarn("No device %s found, sleeping" % device_path)
        rospy.sleep(5)
    dev = evdev.InputDevice(device_path)
    dev.grab()

    x, y = 0, 0
    vp = 'center'

    n = 0
    while not rospy.is_shutdown():
        try:
            ev = dev.read_one()
            if ev is None:
                raise IOError
        except IOError:
            rospy.sleep(sleep_time)
            n += 1
            # if idle for long enough, reset position to center
            if n >= mouse_timeout / sleep_time:
                x = 0
                y = 0
                n = 0
            continue
        n = 0

        ev_type = ev.type
        ev_code = ev.code
        ev_value = ev.value

        if ev_type == ecodes.EV_KEY:
            if ev_code == ecodes.KEY_INSERT and ev_value != 0:
                x = 0
                y = 0
                continue
            events_msg = EvdevEvents()
            events_msg.events.append(EvdevEvent(
                type=ev_type,
                code=ev_code,
                value=ev_value
            ))
            kbd_events_pub.publish(events_msg)
            continue
        elif ev_type == ecodes.EV_REL:
            if ev_code == ecodes.REL_X:
                x += ev_value
            elif ev_code == ecodes.REL_Y:
                y += ev_value
        else:
            continue

        ang_x = math.radians(x / 80.0)
        ang_y = math.radians(y / 80.0)
        new_vp, vpx, vpy = mvp.orientation_to_coords(ang_x, ang_y)

        if vp != new_vp:
            routes_msg = StringArray(strings=[new_vp])
            routes_pub.publish(routes_msg)
            vp = new_vp

        events_msg = EvdevEvents()

        events_msg.events.append(EvdevEvent(
            type=ecodes.EV_ABS,
            code=ecodes.ABS_X,
            value=vpx * DEV_WIDTH,
        ))
        events_msg.events.append(EvdevEvent(
            type=ecodes.EV_ABS,
            code=ecodes.ABS_Y,
            value=vpy * DEV_HEIGHT,
        ))

        events_pub.publish(events_msg)


if __name__ == '__main__':
    main()
