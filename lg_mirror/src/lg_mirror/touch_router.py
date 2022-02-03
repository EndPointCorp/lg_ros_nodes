import evdev
import re
import rospy
import threading

from .constants import MIRROR_ACTIVITY_TYPE
from .constants import MIRROR_TOUCH_CONFIG_KEY
from lg_common.helpers import route_touch_to_viewports
from lg_common.managed_window import ManagedWindow
from lg_msg_defs.msg import RoutedEvdevEvents
from lg_msg_defs.srv import EvdevDeviceInfo


def absolute_geometry(window):
    geometry = ManagedWindow.lookup_viewport_geometry(window['presentation_viewport'])
    geometry.width = window['width']
    geometry.height = window['height']
    geometry.x += window['x_coord']
    geometry.y += window['y_coord']
    return geometry


def is_point_in_rect(x, y, rect):
    return (
        x >= rect.x and x <= rect.x + rect.width
        and y >= rect.y and y <= rect.y + rect.height
    )


def is_point_in_rects(x, y, rects):
    return any((is_point_in_rect(x, y, rect) for rect in rects))


def is_touch_btn_event(event):
    # EV_KEY BTN_TOUCH
    return event.type == 0x01 and event.code == 0x14a


class SubscribeListener:
    def __init__(self, publish_callback):
        self.publish_callback = publish_callback

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.logdebug("New subscription. %s / %s / %s" % (
            topic_name,
            self.publish_callback,
            self.publish_callback)
        )

        self.publish_callback(topic_name)

    def peer_unsubscribe(self, topic_name, num_peers):
        pass


class TouchRouter:
    def __init__(self, event_pub, default_viewport=None):
        self.event_pub = event_pub
        if default_viewport is None:
            self.default_viewports = set()
        else:
            self.default_viewports = set([default_viewport])

        self.route_viewports = self.default_viewports
        self.touchmenu_visible = True
        self.touchmenu_geometry = ManagedWindow.lookup_viewport_geometry('touchscreen')
        self.wall_geometry = ManagedWindow.lookup_viewport_geometry('wall_a')
        self.spacenav_mode = False
        self.spacenav_exclusion_rects = []
        self.spacenav_viewport = 'fake_wall_a'
        self.spacenavving = False
        self.non_multitouch_activities = [
            "streetview",
            "cesium",
            "unity",
            "panovideo",
        ]
        self.lock = threading.Lock()

        svc_name = f"/lg_mirror/default/device_info"
        rospy.wait_for_service(svc_name)
        svc = rospy.ServiceProxy(svc_name, EvdevDeviceInfo)
        self.touchscreen = svc()

    def handle_service_request(self, req):
        """
        Returns a string[] of active viewports
        """
        with self.lock:
            if len(self.route_viewports) == 0:
                return self.default_viewports
            else:
                return self.route_viewports


    def handle_touchmenu_state(self, msg):
        """
        Handles touchmenu state change so we can retain its state.
        """
        with self.lock:
            self.touchmenu_visible = msg.data
            rospy.loginfo(f'touchmenu visible: {self.touchmenu_visible}')
            if self.touchmenu_visible:
                self.touchmenu_geometry = ManagedWindow.lookup_viewport_geometry('touchscreen')

    def handle_touch_event(self, msg):
        routed = RoutedEvdevEvents(events=msg.events)
        try:
            with self.lock:
                if len(self.route_viewports) == 0:
                    routed.routes = self.default_viewports
                else:
                    routed.routes = self.route_viewports

                if not self.spacenavving and not self.spacenav_mode:
                    return  # Bypass re-routing when not in spacenav mode or already re-routed.

                events = msg.events
                button = next((e for e in events if is_touch_btn_event(e)), None)
                if button is None:
                    if self.spacenavving:
                        routed.routes = [self.spacenav_viewport]  # Continue routing to nav viewport.
                    return
                elif button.value == 0:
                    if self.spacenavving:
                        routed.routes = [self.spacenav_viewport]  # Route last message to nav viewport.
                    self.spacenavving = False
                    return

                # Hereafter we are handling a new touch, figure out where to route it.
                x_scale = self.wall_geometry.width / (self.touchscreen.abs_max[0] - self.touchscreen.abs_min[0])
                y_scale = self.wall_geometry.height / (self.touchscreen.abs_max[1] - self.touchscreen.abs_min[1])
                x, y = None, None
                for event in events:
                    if event.type == 0x03:  # EV_ABS
                        if event.code == 0x00:  # ABS_X
                            x = event.value * x_scale
                        elif event.code == 0x01:  # ABS_Y
                            y = event.value * y_scale

                rects = self.spacenav_exclusion_rects.copy()
                if self.touchmenu_visible:
                    rospy.loginfo('adding touchmenu rect')
                    rects.append(self.touchmenu_geometry)

                if is_point_in_rects(x, y, rects):
                    rospy.loginfo('exclusion')
                else:
                    # Route all events to nav until this touch ends.
                    self.spacenavving = True
                    routed.routes = [self.spacenav_viewport]
                    rospy.loginfo('no exclusion')
        finally:
            self.event_pub.publish(routed)

    def handle_scene(self, publish_cb, scene):
        """
        Handles an incoming director scene by publishing the list of viewports
        that should have touches routed to them.

        Args:
            publish_cb (function): Callback for publishing the list of
                viewports.
            scene (dict): Director scene.
        """
        with self.lock:
            self.spacnav_mode = False
            self.spacenav_exclusion_rects = []
            windows = scene.get('windows', [])
            route_viewports = route_touch_to_viewports(windows, route_touch_key=MIRROR_TOUCH_CONFIG_KEY)
            self.route_viewports = route_viewports

            if len(route_viewports) > 0:
                # Should this be the route during spacenav exclusion?  Probably..
                rospy.loginfo(f'routing to specific viewports: {route_viewports}')
                publish_cb(frozenset(route_viewports))
                return

            if not any((w['activity'] in self.non_multitouch_activities for w in windows)):
                # It's Earth!  At least a little bit.
                self.spacenav_mode = True
                rects = [absolute_geometry(w) for w in windows if w['activity'] != 'earth']
                rects = [g for g in rects if g is not None]
                rects.append(ManagedWindow.lookup_viewport_geometry('touchscreen_button'))
                self.spacenav_exclusion_rects = rects
                #rospy.loginfo(f'routing to spacenav: {self.spacenav_viewport}')
                #publish_cb(frozenset([self.spacenav_viewport]))
                return

            rospy.loginfo(f'routing to default viewports: {self.default_viewports}')
            publish_cb(frozenset(self.default_viewports))

    def handle_new_listener(self, publish_cb, data):
        """
        Handles new listener connection

        Args:
            publish_cb (function): Callback for publishing the list of
                viewports.
            data: data about new listener
        """
        with self.lock:
            rospy.loginfo("New listener %s" % data)

            if len(self.route_viewports) == 0:
                self.route_viewports = self.default_viewports

            publish_cb(self.route_viewports)
