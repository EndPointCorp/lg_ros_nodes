from threading import Lock

from evdev import UInput, ecodes

# We are emulating an ELO touchscreen with the following min/max values.
MIN_X = 0
MAX_X = 4095
MIN_Y = 0
MAX_Y = 4095


class FakeTouchscreen:
    """
    Uses the Linux multi-touch protocol and uinput to create a virtual
    touchscreen.

    A type A (anonymous contacts) device is simulated.

    https://www.kernel.org/doc/Documentation/input/multi-touch-protocol.txt
    """
    def __init__(self, device_info):
        """
        Create a virtual touchscreen with the capabilities of a typical
        ELO touchscreen.
        """
        self._lock = Lock()

        event_codes = {
            ecodes.EV_SYN: [ecodes.SYN_REPORT, ecodes.SYN_MT_REPORT],
            ecodes.EV_KEY: [ecodes.BTN_TOUCH],
            ecodes.EV_ABS: [
                ecodes.ABS_X,
                ecodes.ABS_Y,
                ecodes.ABS_MT_SLOT,
                ecodes.ABS_MT_POSITION_X,
                ecodes.ABS_MT_POSITION_Y,
                ecodes.ABS_MT_TRACKING_ID,
            ],
        }

        self.device = UInput(
            event_codes,
            vendor=0x04e7,
            product=0x00c0,
            version=0x0111,
            bustype=3,
            name="Fake Touchscreen"
        )

    def touch(self, coord):
        """
        Writes a single touch.

        A lg_mirror.msg.ScreenCoordinate is expected.
        """
        with self._lock:
            self.device.write(ecodes.EV_ABS, ecodes.ABS_MT_POSITION_X, coord.x)
            self.device.write(ecodes.EV_ABS, ecodes.ABS_MT_POSITION_Y, coord.y)
            self.device.write(ecodes.EV_SYN, ecodes.SYN_MY_REPORT, 0)
            self.device.syn()
