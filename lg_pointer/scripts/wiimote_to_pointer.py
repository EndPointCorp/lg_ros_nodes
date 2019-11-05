#!/usr/bin/env python3

from threading import Lock
import math
from evdev import ecodes
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


def handle_stale_device():
    """
    When the angular velocity data hasn't changed for a while, we know that
    the controller is no longer paired, but wiimote_node doesn't.

    By killing wiimote_node we allow it to respawn and pairing can happen again.
    """
    subprocess.check_call(['pkill', '-f', '/opt/ros/[a-z]+/lib/wiimote/wiimote_node'])


class WiiMoteToPointer:
    def __init__(self, events_pub, routes_pub, feedback_pub, imu_calibrate, stale_handler, mvp):
        self.events_pub = events_pub
        self.routes_pub = routes_pub
        self.feedback_pub = feedback_pub
        self.imu_calibrate = imu_calibrate
        self.stale_handler = stale_handler
        self.mvp = mvp
        self.last_state = State()
        self.last_vp = ''
        self.stale_count = 0

        self.z = 0
        self.x = 0
        self.y = 0

        self.vz = 0
        self.vx = 0
        self.vy = 0

        self.was_clicking = False

        self._lock = Lock()

    def tick(self, tev):
        with self._lock:
            try:
                self._tick(tev)
            except Exception as e:
                rospy.logerr(e.message)

    def _tick(self, tev):
        dt = tev.last_duration
        if dt is None or dt > 2.0 / TICK_RATE:
            dt = 0

        msg = self.last_state

        clicking = msg.buttons[4]
        moving = msg.buttons[5]

        if moving:
            self.y = self.y + self.vy * dt * TICK_RATE
            sy = math.sin(self.y)
            cy = math.cos(self.y)
            self.z = self.z + (self.vz * cy - self.vx * sy) * dt * TICK_RATE
            self.x = self.x + (self.vx * cy + self.vz * sy) * dt * TICK_RATE
        else:
            self.z = 0
            self.x = 0
            self.y = 0

        if not self.mvp.clamp(self.z, self.x):
            vp, vpx, vpy = self.mvp.orientation_to_coords(self.z, self.x)

        if moving and vp != self.last_vp:
            routes_msg = StringArray(strings=[vp])
            self.routes_pub.publish(routes_msg)
            self.last_vp = vp

        # bypass event if no viewport -- it would not be routed anyway
        if vp == '':
            return

        events_msg = EvdevEvents()

        if moving and not self.was_clicking:
            events_msg.events.append(EvdevEvent(
                type=ecodes.EV_ABS,
                code=ecodes.ABS_X,
                value=vpx * DEV_WIDTH
            ))
            events_msg.events.append(EvdevEvent(
                type=ecodes.EV_ABS,
                code=ecodes.ABS_Y,
                value=vpy * DEV_HEIGHT
            ))

        if clicking and not self.was_clicking:
            events_msg.events.append(EvdevEvent(
                type=ecodes.EV_KEY,
                code=ecodes.BTN_LEFT,
                value=1
            ))
        elif not clicking and self.was_clicking:
            events_msg.events.append(EvdevEvent(
                type=ecodes.EV_KEY,
                code=ecodes.BTN_LEFT,
                value=0
            ))

        self.was_clicking = clicking

        if len(events_msg.events) > 0:
            self.events_pub.publish(events_msg)

    def handle_wiimote_state(self, msg):
        with self._lock:
            if self.vz == -msg.angular_velocity_zeroed.z and \
               self.vx == msg.angular_velocity_zeroed.x and \
               self.vy == -msg.angular_velocity_zeroed.y:
                self.stale_count += 1
            else:
                self.stale_count = 0

            if self.stale_count == MAX_STALE_COUNT:
                rospy.logwarn('Attempting to reset a stale device')
                self.stale_handler()
                return

            self.vz = -msg.angular_velocity_zeroed.z
            self.vx = msg.angular_velocity_zeroed.x
            self.vy = -msg.angular_velocity_zeroed.y
            self.last_state = msg

            if not msg.LEDs[0]:
                feedback_array_msg = JoyFeedbackArray()
                feedback_msg = JoyFeedback()
                feedback_msg.intensity = 1.0
                feedback_array_msg.array.append(feedback_msg)
                self.feedback_pub.publish(feedback_array_msg)


def main():
    rospy.init_node(NODE_NAME)

    device_id = rospy.get_param('~device_id', 'default')
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

    wmtp = WiiMoteToPointer(events_pub, routes_pub, feedback_pub,
                            imu_calibrate, handle_stale_device, mvp)

    rospy.Subscriber('/wiimote/state', State, wmtp.handle_wiimote_state)

    rospy.Service(info_srv, EvdevDeviceInfo, handle_device_info)

    rospy.Timer(rospy.Duration(1.0 / TICK_RATE), wmtp.tick)

    rospy.spin()


if __name__ == '__main__':
    main()
