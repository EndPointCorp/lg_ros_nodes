from geometry_msgs.msg import Twist
from lg_common.msg import ApplicationState
from evdev import AbsInfo, UInput, InputEvent, ecodes as e
import time


class DeviceWriter:
    def __init__(self, scale):
        self.scale = scale
        # most values were taken from running
        # InputDevice('/dev/input/event$N').capabilities()
        vendor = 1133
        product = 0
        version = 273
        bustype = 3
        caps = {
            e.EV_KEY: [e.BTN_0, e.BTN_1],
            e.EV_REL: [e.REL_X, e.REL_Y, e.REL_Z, e.REL_RX, e.REL_RY, e.REL_RZ],
        }
        device_name = 'Virtual SpaceNav'
        self.state = True
        self.ui = UInput(caps, vendor=vendor, product=product,
                         version=version, bustype=bustype, name=device_name)

    def make_event(self, data):
        if not self.state:
            return
        for v in (data.linear, data.angular):
            # We don't want to write zeroes because the real spacenav never does,
            # instead write a small value inside Earth's gutter to prevent drift.
            if v.x == 0:
                v.x = 1.0 / self.scale
            if v.y == 0:
                v.y = 1.0 / self.scale
            if v.z == 0:
                v.z = 1.0 / self.scale
        # write some event to self.ui based off of the twist data
        _time = time.time()
        stime = int(_time)
        utime = int((float(_time) - stime) * 10 ** 6)
        x = InputEvent(stime, utime, e.EV_REL, e.REL_X, -self.translate(data.linear.y))
        y = InputEvent(stime, utime, e.EV_REL, e.REL_Y, -self.translate(data.linear.x))
        z = InputEvent(stime, utime, e.EV_REL, e.REL_Z, -self.translate(data.linear.z))
        ax = InputEvent(stime, utime, e.EV_REL, e.REL_RX, -self.translate(data.angular.y))
        ay = InputEvent(stime, utime, e.EV_REL, e.REL_RY, -self.translate(data.angular.x))
        az = InputEvent(stime, utime, e.EV_REL, e.REL_RZ, -self.translate(data.angular.z))
        # write all events
        self.ui.write_event(x)
        self.ui.write_event(y)
        self.ui.write_event(z)
        self.ui.write_event(ax)
        self.ui.write_event(ay)
        self.ui.write_event(az)
        # syn to alert input subsystem
        self.ui.syn()

    def translate(self, n):
        return int(n * self.scale)  # TODO find the translation...

    def set_state(self, msg):
        self.state = msg.state == ApplicationState.VISIBLE
