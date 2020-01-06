#!/usr/bin/env python3

import rospy
from lg_common import ManagedBrowser
from lg_msg_defs.msg import ApplicationState
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('browser')

    browser = ManagedBrowser(
        url='https://www.google.com',
        force_device_scale_factor=2,
        user_agent='Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; ' +
                   'en-us) AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
                   'Version/4.0.4 Mobile/7B314 Safari/531.21.10',
        remote_debugging_port=10000,
    )

    # Start the process and converge its window.
    browser.set_state(ApplicationState.VISIBLE)

    # Provide a state topic for debugging.
    rospy.Subscriber('/example_browser/state', ApplicationState,
                     browser.handle_state_msg)

    # Provide a debug socket topic for debugging.
    # i.e. {"id":1,"method":"Page.reload","params":{"ignoreCache":"True"}}
    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)
    rospy.Subscriber('/example_browser/debug', String, handle_debug_sock_msg)

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
