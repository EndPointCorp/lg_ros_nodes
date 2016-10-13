#!/usr/bin/env python
import rospy
import json

from lg_common.srv import USCSMessage
from std_msgs.msg import String
from appctl.msg import Mode


class StateSetter(object):
    def __init__(self, state_pub, display_url_pub, kiosk_url_pub, runway_pub, get_mode):
        self.state_pub = state_pub
        self.display_url_pub = display_url_pub
        self.kiosk_url_pub = kiosk_url_pub
        self.runway_pub = runway_pub
        self.get_mode = get_mode
        self.state_42_a = None
        self.state_42_b = None
        self.state = None

    def handle_state_setting(self, msg):
        self.state = None
        try:
            state = json.loads(msg.data)
        except Exception:
            rospy.logerr('Error with the state message, non json format:\n%s' % msg.data)
            return
        # if we are not in the current state, we need to set the state and
        # wait for the browser to come up and query us through the service
        # /state_setter/desired_state
        if str(state['mode']) != self.get_mode().mode:
            self.state_pub.publish(state['mode'])
            self.state = self.state_42_a = self.state_42_b = state
            return
        if state['mode'] == 'tactile':
            self.handle_tactile(state)
        else:
            self.publish_urls(state['kiosk_url'], state['display_url'])
        # set state to none since we don't need to store it when
        # we change the url / handle tactile ourself
        self.state = self.state_42_a = self.state_42_b = None

    def desired_state(self, req):
        state = self.state
        if state is None:
            return ''
        if req.node == '42-a' or req.node == 'display':
            if self.state_42_a is None:
                return ''
            self.state_42_a = None
            return json.dumps(state)
        if req.node == '42-b' or req.node == 'kiosk':
            if self.state_42_b is None:
                return ''
            self.state_42_b = None
            return json.dumps(state)
        return ''

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


def main():
    rospy.init_node('state_setter')
    state_pub = rospy.Publisher('/director/scene', Mode, queue_size=10)
    runway_pub = rospy.Publisher('/portal_kiosk/runway_change', String, queue_size=10)
    state_setter = StateSetter(state_pub, runway_pub, get_mode)

    rospy.Service('/state_setter/desired_state', DesiredState, state_setter.desired_state)
    rospy.Subscriber('/state_setter/set_state', String, state_setter.handle_state_setting)
    rospy.spin()

if __name__ == '__main__':
    main()
