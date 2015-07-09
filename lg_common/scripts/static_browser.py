#!/usr/bin/env python

import rospy
from lg_common import ManagedBrowser
from lg_common.msg import ApplicationState
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('static_browser')

    url = rospy.get_param('~url', None)
    viewport = rospy.get_param('~viewport', None)

    user_agent = rospy.get_param(
        '~user_agent', 'Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; '
        'en-us AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
        'Version/4.0.4 Mobile/7B314 Safari/531.21.10'
    )
    debug_port = rospy.get_param('~debug_port', 10000),

    state = rospy.get_param('~state', ApplicationState.VISIBLE)

    browser = ManagedBrowser(
        url=url,
        force_device_scale_factor=2,
        user_agent=user_agent,
        debug_port=debug_port,
    )

    browser.set_state(state)

    rospy.Subscriber('{}/state'.format(rospy.get_name()), ApplicationState,
                     browser.handle_state_msg)

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('/example_browser/debug', String, handle_debug_sock_msg)

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
