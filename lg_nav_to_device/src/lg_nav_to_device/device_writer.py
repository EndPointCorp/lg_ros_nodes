from geometry_msgs.msg import Twist
from evdev import AbsInfo, UInput, InputEvent, ecodes as e
import time

class DeviceWriter:
    def __init__(self, scale):
        self.scale = scale
        # most values were taken from running
        # InputDevice('/dev/input/event$N').capabilities()
        vendor=1133
        product = 0
        version = 273
        bustype = 3
        common_abs = AbsInfo(value=0, min=0, max=32767, fuzz=0, flat=0,
                             resolution=0)
        tracking_abs = AbsInfo(value=0, min=0, max=65535, fuzz=0, flat=0,
                               resolution=0)
        slot_abs = AbsInfo(value=0, min=0, max=9, fuzz=0, flat=0, resolution=0)
        spacenav_events = {
            1L: [256L, 257L],
            2L: [0L, 1L, 2L, 3L, 4L, 5L],
            # the abs values were stolen from the previous hardcoding in
            # evdev_teleport, for some reason device.capabilities() did not
            # show any abs values...
            3L: [(e.ABS_X, common_abs),
                 (e.ABS_Y, common_abs),
                 (e.ABS_MT_POSITION_X, common_abs),
                 (e.ABS_MT_POSITION_Y, common_abs),
                 (e.ABS_MT_TRACKING_ID, tracking_abs),
                 (e.ABS_MT_SLOT, slot_abs)
                ],
            4L: [4L],
            17L: [8L]
        }
        device_name = 'Virtual SpaceNav'
        self.ui = UInput(spacenav_events, vendor=vendor, product=product,
                         version=version, bustype=bustype, name=device_name)

    def make_event(self, data):
        # write some event to self.ui based off of the twist data
        _time = time.time()
        stime = int(_time)
        utime = int((float(_time) - stime) * 10 ** 6)
        # linear and angular might need to be switched here...
        x = InputEvent(stime, utime, e.EV_REL, e.REL_X, self.translate(data.linear.x))
        y = InputEvent(stime, utime, e.EV_REL, e.REL_Y, self.translate(data.linear.y))
        z = InputEvent(stime, utime, e.EV_REL, e.REL_Z, self.translate(data.linear.z))
        ax = InputEvent(stime, utime, e.EV_REL, e.REL_RX, self.translate(data.angular.x))
        ay = InputEvent(stime, utime, e.EV_REL, e.REL_RY, self.translate(data.angular.y))
        az = InputEvent(stime, utime, e.EV_REL, e.REL_RZ, self.translate(data.angular.z))
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
        return int(n * self.scale) # TODO find the translation...
