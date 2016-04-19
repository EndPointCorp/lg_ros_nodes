#!/usr/bin/env python

import rospy

from lg_common.helpers import get_params
from lg_common import ManagedBrowser, ManagedWindow
from lg_common.msg import ApplicationState, WindowGeometry
from lg_common.helpers import dependency_available, DependencyException
from lg_common.helpers import discover_host_from_url, discover_port_from_url

from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('static_browser')

    geometry = ManagedWindow.get_viewport_geometry()
    url = get_params('~url', None)
    command_line_args = get_params('~command_line_args', '')
    scale_factor = get_params('~force_device_scale_factor', 1)
    debug_port = get_params('~debug_port', 10000)
    user_agent = get_params(
        '~user_agent', 'Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; '
        'en-us AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
        'Version/4.0.4 Mobile/7B314 Safari/531.21.10'
    )
    state = get_params('~state', ApplicationState.VISIBLE)
    extensions = get_params('~extensions', [])

    global_dependency_timeout = get_params("/global_dependency_timeout", 15)
    depend_on_url = get_params("~depend_on_url", False)

    www_host = discover_host_from_url(url)
    www_port = discover_port_from_url(url)

    if depend_on_url:
        if not dependency_available(www_host, www_port, 'static browser URL', global_dependency_timeout):
            msg = "Service: %s hasn't become accessible within %s seconds" % ('director', global_dependency_timeout)
            rospy.logfatal(msg)
            raise DependencyException(msg)
        else:
            rospy.loginfo("URL available - continuing initialization")

    browser = ManagedBrowser(
        geometry=geometry,
        url=url,
        command_line_args=command_line_args,
        force_device_scale_factor=scale_factor,
        debug_port=debug_port,
        user_agent=user_agent,
        extensions=extensions
    )

    browser.set_state(state)

    rospy.Subscriber('{}/state'.format(rospy.get_name()), ApplicationState,
                     browser.handle_state_msg)

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('{}/debug'.format(rospy.get_name()), String, handle_debug_sock_msg)

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
