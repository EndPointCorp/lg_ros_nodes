#!/usr/bin/env python

from threading import Lock
import math
from evdev import ecodes
import evdev
import subprocess

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
    res.key_codes = [ecodes.BTN_LEFT]
    res.abs_min = [0, 0]
    res.abs_max = [DEV_WIDTH, DEV_HEIGHT]
    return res


def main():
    rospy.init_node(NODE_NAME)

    device_id = rospy.get_param('~device_id', 'default')
    device_path = rospy.get_param('~device_path', 'default')
    viewports = [
        v.strip() for v in rospy.get_param('~viewports').split(',')
    ]
    arc_width = float(rospy.get_param('~arc_width', math.pi / 2))

    events_topic = '/lg_mirror/{}/events'.format(device_id)
    routes_topic = '/lg_mirror/{}/active_routes'.format(device_id)
    info_srv = '/lg_mirror/{}/device_info'.format(device_id)

    events_pub = rospy.Publisher(events_topic,
                                 EvdevEvents, queue_size=10)
    routes_pub = rospy.Publisher(routes_topic,
                                 StringArray, queue_size=10)
    feedback_pub = rospy.Publisher('/joy/set_feedback',
                                   JoyFeedbackArray, queue_size=10)
    imu_calibrate = rospy.ServiceProxy('/imu/calibrate', Empty, persistent=True)

    mvp = MegaViewport(viewports, arc_width)

    rospy.Service(info_srv, EvdevDeviceInfo, handle_device_info)

    dev = evdev.InputDevice(device_path)
    dev.grab()

    x, y = 0, 0
    vp = 'center'
    #for ev in dev.read_loop():
    while not rospy.is_shutdown():
        try:
            ev = dev.read_one()
        except IOError:
            rospy.sleep(0.1)
            continue
        ev_type = ev.type
        ev_code = ev.code
        ev_value = ev.value
        print ev

        if ev_type == ecodes.EV_KEY:
            if ev_code == ecodes.KEY_INSERT and ev_value != 0:
                x = 0
                y = 0
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
        print 'x: {:.4f} y: {:.4f}'.format(ang_x, ang_y)
        new_vp, vpx, vpy = mvp.orientation_to_coords(ang_x, ang_y)
        print 'vp: {} x: {} y: {}'.format(new_vp, vpx, vpy)

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


        """
        val = 0
        rospy.logerr("x: {}, y: {}, vpx: {}, vpy: {}".format(
            x, y, vpx, vpy
        ))
        if ev.code == ecodes.REL_X:
            rospy.logerr("tis rel_x")
            val = x
        elif ev.code == ecodes.REL_Y:
            rospy.logerr("tis rel_y")
            val = y

        if new_vp != vp:
            vp = new_vp
            routes_pub.publish(StringArray(strings=['center']))

        events_msg = EvdevEvents()
        events_msg.events.append(EvdevEvent(
            type=ev.type,
            code=ev.code,
            value=val
        ))
        events_pub.publish(events_msg)
        """

    #rospy.spin()


if __name__ == '__main__':
    main()
