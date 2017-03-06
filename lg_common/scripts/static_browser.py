#!/usr/bin/env python

import rospy

from lg_common import ManagedBrowser, ManagedWindow
from lg_common.msg import ApplicationState, WindowGeometry
from lg_common.helpers import check_www_dependency, discover_host_from_url, discover_port_from_url
from lg_common.helpers import run_with_influx_exception_handler
from std_msgs.msg import String


NODE_NAME = 'static_browser'


def main():
    rospy.init_node(NODE_NAME)

    geometry = ManagedWindow.get_viewport_geometry()
    url = rospy.get_param('~url', None)
    command_line_args = rospy.get_param('~command_line_args', '')
    scale_factor = rospy.get_param('~force_device_scale_factor', 1)
    extra_logging = rospy.get_param('~extra_logging', False)
    debug_port = rospy.get_param('~debug_port', None)
    user_agent = rospy.get_param(
        '~user_agent', 'Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; '
        'en-us AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
        'Version/4.0.4 Mobile/7B314 Safari/531.21.10'
    )
    state = rospy.get_param('~state', ApplicationState.VISIBLE)
    extensions = rospy.get_param('~extensions', [])
    kiosk = rospy.get_param('~kiosk', True)

    global_dependency_timeout = rospy.get_param("/global_dependency_timeout", 15)
    depend_on_url = rospy.get_param("~depend_on_url", False)

    www_host = discover_host_from_url(url)
    www_port = discover_port_from_url(url)
    check_www_dependency(depend_on_url, www_host, www_port, 'static browser URL', global_dependency_timeout)

    browser = ManagedBrowser(
        geometry=geometry,
        url=url,
        command_line_args=command_line_args,
        log_stderr=extra_logging,
        force_device_scale_factor=scale_factor,
        remote_debugging_port=debug_port,
        user_agent=user_agent,
        extensions=extensions,
        kiosk=kiosk
    )

    browser.set_state(state)

    rospy.Subscriber('{}/state'.format(rospy.get_name()), ApplicationState,
                     browser.handle_state_msg)

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('{}/debug'.format(rospy.get_name()), String, handle_debug_sock_msg)

    rospy.spin()

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
