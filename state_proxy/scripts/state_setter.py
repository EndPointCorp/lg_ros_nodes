#!/usr/bin/env python3
import rospy
import json

from lg_common.srv import USCSMessage
from state_proxy.srv import DesiredState
from interactivespaces_msgs.msg import GenericMessage
from std_msgs.msg import String
from appctl.msg import Mode
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'state_setter'


class StateSetter(object):
    def __init__(self, state_pub, display_url_pub, kiosk_url_pub, runway_pub, last_uscs_service):
        self.state_pub = state_pub
        self.display_url_pub = display_url_pub
        self.kiosk_url_pub = kiosk_url_pub
        self.runway_pub = runway_pub
        self.last_uscs_service = last_uscs_service
        self.state_display = None
        self.state_kiosk = None
        self.state = None

    def get_current_state(self):
        state = self.last_uscs_service().message
        try:
            return json.loads(state)
        except Exception:
            rospy.logerr("Last state from /uscs/message service returned non-json parsable (%s)" % state)
            return {}

    def handle_state_setting(self, msg):
        self.state = None
        try:
            state = json.loads(msg.data)
        except Exception:
            rospy.logerr('Error with the state message, non json format:\n%s' % msg.data)
            return

        self.state = state

        # if the current state is tactile and the new state is tactile, then
        # we follow a special path, also if just the new state is tactile we
        # follow another but different special path

        #if self.handle_current_and_new_tactile(state):
        #    self._clear_state()
        #    return

        #if self.handle_new_state_tactile(state):
        #    return

        self.publish_uscs(state)
        self._clear_state()

    def _clear_state(self):
        # set state to none since we don't need to store it when
        # we change the url / handle tactile ourself
        self.state = self.state_display = self.state_kiosk = None

    def handle_current_and_new_tactile(self, new_state):
        # if the current and new state are tactile, only urls / runway
        # cards need to be changed / emitted

        current_state = self.get_current_state()
        if not self.is_tactile(current_state) or not self.is_tactile(new_state):
            return False

        if self.valid_runway_card(state.get('runway_card', None)):
            self.runway_pub.publish(state['runway_card'])
        else:
            # if the runway card isn't valid we want to just set the urls
            self.kiosk_pub.publish(self.get_kiosk_url(state))
            self.display_pub.publish(self.get_display_url(state))

        return True

    def handle_new_state_tactile(self, new_state):
        self.publish_uscs(new_state)
        # store state so when the kiosk and display are finished
        # loading they can query their own runway cards
        self.state = self.state_kiosk = self.state_display = new_state

    def publish_uscs(self, state):
        self.state_pub.publish(self.make_director(state))

    def desired_state(self, req):
        state = self.state
        if state is None:
            return ''
        if req.node == '42-a' or req.node == 'display':
            if self.state_display is None:
                return ''
            self.state_display = None
            return json.dumps(state)
        if req.node == '42-b' or req.node == 'kiosk':
            if self.state_kiosk is None:
                return ''
            self.state_kiosk = None
            return json.dumps(state)
        return ''

    def make_director(self, uscs_message):
        # makes a generic message and returns it
        ret = GenericMessage()
        ret.type = 'json'
        try:
            ret.message = json.dumps(uscs_message)
        except Exception:
            rospy.logerr('Could not dump state message into json...')
            ret.message = ''
        return ret

    def valid_runway_card(self, runway_card):
        # runway cards can sometimes be "None" as a string
        if runway_card is None and runway_card == 'None':
            return False
        if runway_card[11] == '3':
            return False
        return True

    def handle_tactile(self, new_state):
        if new_state.get('runway_card', 'None') != 'None' and \
                new_state.get('runway_card') is not None and \
                new_state.get('runway_card')[11] != '3':
            self.runway_pub.publish(new_state['runway_card'])
            return
        self.publish_urls(new_state['kiosk_url'], new_state['display_url'])

    def publish_urls(self, kiosk_url, display_url):
        self.kiosk_url_pub.publish(kiosk_url)
        self.display_url_pub.publish(display_url)

    def grab_urls(self, state):
        # grabs urls which are the only asset when the
        # activity is "browser" If there are more assets
        # then we ignore the window
        urls = []
        for window in state.get('windows', []):
            if window.get('activity') != 'browser':
                continue
            if len(window.get('assets', [])) == 1:
                urls.append(window['assets'][0])
        return urls

    def _is_tactile_url(self, urls):
        # checking that the length of the filter is not zero, if it is then no urls
        # matched those that should be tactile
        return len([url for url in urls if 'maps.google.com' in url or 'google.com/maps' in url]) != 0

    def is_tactile(self, state):
        self._is_tactile_url(self.grab_urls(state))


def main():
    rospy.init_node(NODE_NAME)
    state_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
    runway_pub = rospy.Publisher('/portal_kiosk/runway_change', String, queue_size=10)
    display_pub = rospy.Publisher('/display/switch', String, queue_size=10)
    kiosk_pub = rospy.Publisher('/kiosk/switch', String, queue_size=10)

    rospy.wait_for_service('/uscs/message', 10)
    last_uscs_service = rospy.ServiceProxy('/uscs/message', USCSMessage)

    state_setter = StateSetter(state_pub, display_pub, kiosk_pub, runway_pub, last_uscs_service)

    rospy.Service('/state_setter/desired_state', DesiredState, state_setter.desired_state)
    rospy.Subscriber('/state_setter/set_state', String, state_setter.handle_state_setting)
    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
