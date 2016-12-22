#!/usr/bin/env python
import rospy
import json

from lg_common.srv import BrowserPool, USCSMessage
from lg_common.helpers import add_url_params
from std_msgs.msg import String
from appctl.msg import Mode
from urllib2 import urlopen
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'state_tracker'

class StateTracker(object):
    def __init__(self, state_publisher, update_rfid_pub, last_uscs_service,
                 tactile_flag='', display_url_service=None, kiosk_url_service=None):
        self.state_publisher = state_publisher
        self.update_rfid_pub = update_rfid_pub
        self.last_uscs_service = last_uscs_service
        self.last_runway_card = None
        self.ignore_card = 'click!![1,[3,null,[true],[null,null,0],null,null,false,1],1]'
        self.exit_card = 'exit!![1,[],0]'
        self.last_rfid = ''
        self.tactile_flag = tactile_flag
        self.display_url_service = display_url_service
        self.kiosk_url_service = kiosk_url_service

    def handle_runway_cards(self, msg):
        if msg.data == self.exit_card or msg.data == self.ignore_card or msg.data[11] == '3':
            self.last_runway_card = None
            return
        self.last_runway_card = msg.data

    def build_state(self):
        """
        Calls state tracking service and handles all url grabbing
        """
        current_state = self.last_uscs_service().message
        try:
            current_state = json.loads(current_state)
        except:
            rospy.logerr("Error parsing last uscs message as json")
            return

        windows = current_state.get('windows', [])
        for window in windows:
            url = self.grab_url(window)
            if url is None:
                continue
            # adding cms_protocol and cms_port for the portal launcher
            # only needed on the kiosk
            if window.get('presentation_viewport', None) == 'kiosk':
                # default port and protocol
                protocol = 'http'
                port = '8088'
                if 'https' in url:
                    protocol = 'https'
                    port = '443'
                window['assets'] = [add_url_params(url, cms_protocol=protocol, cms_port=port)]

        current_state = self.handle_tactile(current_state)
        return current_state

    def handle_tactile(self, state):
        for window in state.get('windows', []):
            if window.get('activity', '') != 'browser':
                continue

            for i in range(len(window.get('assets', []))):
                if ('maps.google.com' in window['assets'][i] or "google.com/maps" in window['assets'][i]) and \
                        self.tactile_flag not in window['assets'][i]:
                    # add a param to be sure there is at least one param (HACK)
                    url = add_url_params(window['assets'][i], foo='bar')
                    url += '&%s' % self.tactile_flag
                    window['assets'][i] = url
                # adding cms_protocol and cms_port for the portal launcher
                # only needed on the kiosk
                #if window.get('presentation_viewport', None) == 'kiosk':
                #    window['assets'][i] = add_url_params(window['assets'][i], cms_protocol='https', cms_port='443')

        return state

    def grab_url(self, window):
        """
        given a window (from the director message) grab either the kiosk or
        the display current url based on the viewport from the window. Apply
        any tactile changes if maps.google.com is part of the url
        """
        url_service = None
        activity = window.get('activity', None)
        if activity != 'browser':
            return

        viewport = window.get('presentation_viewport', None)
        if viewport is None:
            rospy.info("viewport was None... ignoring")
            return

        # display might be ok to go away, but only once we're sure
        if viewport != 'kiosk' and viewport != 'wall' and viewport != 'display':
            rospy.warn("Unable to determine viewport named (%s)" % viewport)
            return

        if viewport == 'kiosk':
            url_service = self.kiosk_url_service
        elif viewport == 'display' or viewport == 'wall':
            url_service = self.display_url_service

        state = url_service.call().state
        try:
            state = json.loads(state)
        except:
            rospy.logwarn("Unable to parse state (%s)" % state)
            raise

        if len(state) > 1:
            rospy.logwarn('There are more than one browser active, the wrong URL might be returned')

        for browser_id, browser_data in state.iteritems():
            return browser_data['current_url_normalized']

    def handle_nfc(self, msg):
        self.last_rfid = msg.data
        state = self.build_state()
        state['rfid'] = msg.data
        self.update_rfid_pub.publish(json.dumps(state))


def main():
    rospy.init_node(NODE_NAME)
    current_state_topic = rospy.get_param('~current_state_topic', '/state_tracker/current_state')
    update_rfid_topic = rospy.get_param('~update_rfid_topic', '/rfid/uscs/update')
    tactile_flag = rospy.get_param('~tactile_flag', '')

    # wait for service or kill node
    rospy.wait_for_service('/uscs/message', 10)
    rospy.wait_for_service('/browser_service/wall', 10)
    rospy.wait_for_service('/browser_service/kiosk', 10)

    last_uscs_service = rospy.ServiceProxy('/uscs/message', USCSMessage)
    kiosk_url_service = rospy.ServiceProxy('/browser_service/kiosk', BrowserPool)
    display_url_service = rospy.ServiceProxy('/browser_service/wall', BrowserPool)

    current_state = rospy.Publisher(current_state_topic, String, queue_size=10)
    update_rfid_pub = rospy.Publisher(update_rfid_topic, String, queue_size=10)
    state_tracker = StateTracker(
        current_state, update_rfid_pub, last_uscs_service,
        tactile_flag=tactile_flag, display_url_service=display_url_service,
        kiosk_url_service=kiosk_url_service)

    rospy.Subscriber('/portal_kiosk/runway', String, state_tracker.handle_runway_cards)
    rospy.Subscriber('/rfid/set', String, state_tracker.handle_nfc)

    rospy.spin()

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
