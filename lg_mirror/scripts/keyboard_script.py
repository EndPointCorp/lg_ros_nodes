#!/usr/bin/env python

import rospy
import evdev
from lg_mirror.msg import EvdevEvents
from lg_common.msg import StringArray


class KeyboardThing:
    def __init__(self, routes=None, double_click=-1):
        self.current_route = None
        self.routes = routes
        self.active = False
        self.double_click = int(double_click)
        if self.routes is None:
            self.routes = []
        self.ui = evdev.UInput()

    def handle_event(self, msg):
        for event in msg.events:
            if event.type == 3:
                continue
            if self.active or event.value == 0:  # always publishing key-ups hopefully this doesn't come back to haunt us because honestly it definitely could but thats not something im going to worry about now...
                if event.code == self.double_click and event.value == 1:
                    self.ui.write(1, 272, 1)
                    self.ui.write(1, 272, 0)
                    self.ui.write(1, 272, 1)
                    self.ui.write(1, 272, 0)
                else:
                    self.ui.write(event.type, event.code, event.value)

        self.ui.syn()

    def handle_route_change(self, msg):
        self.current_routes = msg.strings
        for route in self.current_routes:
            if route in self.routes:
                self.active = True
                return
        self.active = False


def main():
    rospy.init_node('keyboard')

    event_topic = rospy.get_param('~event_topic', '/lg_mirror/logitech/')
    routes = rospy.get_param('~routes', '')
    double_click = rospy.get_param('~double_click_key_code', -1)
    routes = routes.split(',')

    kb = KeyboardThing(routes, double_click)

    rospy.logdebug("subscribing to %s and %s with routes %s" % (event_topic + 'active_routes', event_topic + 'events', routes))
    rospy.Subscriber(event_topic + 'active_routes', StringArray, kb.handle_route_change)
    rospy.Subscriber(event_topic + 'kbd_events', EvdevEvents, kb.handle_event)

    rospy.spin()


if __name__ == '__main__':
    main()
