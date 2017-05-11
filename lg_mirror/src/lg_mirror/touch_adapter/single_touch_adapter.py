from threading import Lock
from evdev import ecodes

from lg_mirror.msg import ScreenCoordinate

NULL_VAL = -32768


class DefaultTransformer:
    def transform(self, coord):
        pass


class SingleTouchAdapter:
    def __init__(self, coord_pub, transformer=DefaultTransformer):
        self.coord_pub = coord_pub
        self.transformer = transformer

        self._lock = Lock()

    def handle_touch_events(self, *args, **kwargs):
        with self._lock:
            self._handle_touch_events(*args, **kwargs)

    def _handle_touch_events(self, incoming):
        coord = ScreenCoordinate(x=NULL_VAL, y=NULL_VAL)

        for event in incoming.events:
            if event.type != ecodes.EV_ABS:
                continue

            if event.code == ecodes.ABS_X:
                coord.x = event.value
            elif event.code == ecodes.ABS_Y:
                coord.y = event.value

        if coord.x != NULL_VAL and coord.y != NULL_VAL:
            self.transformer.transform(coord)
            self.coord_pub.publish(coord)
