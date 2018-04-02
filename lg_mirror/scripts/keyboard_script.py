#!/usr/bin/env python

import rospy
import evdev
from lg_mirror.msg import EvdevEvents
from lg_common.msg import StringArray


class KeyboardThing:
    def __init__(self, routes=None):
        self.current_route = None
        self.routes = routes
        self.active = False
        if self.routes is None:
            self.routes = []
        self.ui = evdev.UInput()

    def handle_event(self, msg):
        if not self.active:
            return
        for event in msg.events:
            if event.type != 1:
                continue
            rospy.logerr("Writing event...")
            self.ui.write(event.type, event.code, event.value)

    def handle_route_change(self, msg):
        self.current_routes = msg.strings
        for route in self.current_routes:
            if route in self.routes:
                self.active = True
                rospy.logerr("We are active...")
                return
        self.active = False

def main():
    rospy.init_node('keyboard')

    event_topic = rospy.get_param('~event_topic', '/lg_mirror/logitech/')
    routes = rospy.get_param('~routes', '')
    routes = routes.split(',')

    rospy.logerr("Starting UP THIS NOOOOOODE")
    kb = KeyboardThing(routes)

    rospy.logerr("subscribing to %s and %s with routes %s" % (event_topic + 'active_routes', event_topic + 'events', routes))
    rospy.Subscriber(event_topic + 'active_routes', StringArray, kb.handle_route_change)
    rospy.Subscriber(event_topic + 'events', EvdevEvents, kb.handle_event)

    rospy.spin()

if __name__ == '__main__':
    main()
