from geometry_msgs.msg import Twist
from evdev import AbsInfo, UInput, ecodes as e

class DeviceWriter:
    def __init__(self):
        # most values were taken from running
        # InputDevice('/dev/input/event$N').capabilities()
        vendor=1133
        product = 50726
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
        self.ui = UInput(spacenav_events, vendor=vendor, product=product,
                         version=version, bustype=bustype)

    def make_event(self, data):
        # write some event to self.ui based off of the twist data
        pass

