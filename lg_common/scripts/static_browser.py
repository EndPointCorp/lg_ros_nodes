#!/usr/bin/env python

import rospy
from lg_common import ManagedBrowser
from lg_common.msg import ApplicationState
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('static_browser')

    url = rospy.getparam('~url', None)
    window_x = rospy.getparam('~window_x', None)
    window_y = rospy.getparam('~window_y', None)
    window_w = rospy.getparam('~window_w', None)
    window_h = rospy.getparam('~window_h', None)

    geometry = WindowGeometry(x=window_x, y=window_x,
        width=window_w, height=window_height)

    user_agent = rospy.getparam('~user_agent', 'Mozilla/5.0(iPad; U; ' +
        'CPU iPhone OS 3_2 like Mac OS X; en-us) AppleWebKit/531.21.10 ' +
        '(KHTML, like Gecko) Version/4.0.4 Mobile/7B314 Safari/531.21.10',
    debug_port = rospy.getparam('~debug_port', 10000),

    browser = ManagedBrowser(
        url=url,
        force_device_scale_factor=2,
        user_agent=user_agent,
        debug_port=debug_port,
    )

    browser.set_state(ApplicateionState.VISIBLE)

    rospy.Subscriber('{}/state'.format(rospy.getname()), ApplicationState,
        browser.handle_state_msg)

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('/example_browser/debug', String, handle_debug_sock_msg)

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
