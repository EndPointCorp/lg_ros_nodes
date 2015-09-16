#!/usr/bin/env python

import rospy


from urllib import url2pathname
from std_msgs.msg import String
from lg_common.helpers import add_url_params
from lg_common import ManagedBrowser, ManagedWindow
from lg_common.msg import ApplicationState, WindowGeometry


if __name__ == '__main__':
    rospy.init_node('static_browser')

    geometry = ManagedWindow.get_viewport_geometry()

    url_base = rospy.get_param('~url_base', 'http://lg-head/ros_touchscreens/')
    # TODO (wz) director_host and director_port should be global

    director_host = rospy.get_param('~director_host', '42-a')
    director_port = rospy.get_param('~director_port', 8060)

    rosbridge_secure = rospy.get_param('~rosbridge_secure', 0)
    director_secure = rospy.get_param('~director_secure', 0)

    rosbridge_host = rospy.get_param('~rosbridge_host', '127.0.0.1')
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    ts_name = rospy.get_param('~ts_name', 'default')


    url = url_base + ts_name + "/"

    url = add_url_params(url,
                         director_host=director_host,
                         director_port=director_port,
                         director_secure=director_secure,
                         rosbridge_secure=rosbridge_secure,
                         rosbridge_host=rosbridge_host,
                         rosbridge_port=rosbridge_port)

    url = url2pathname(url)

    scale_factor = rospy.get_param('~force_device_scale_factor', 1)
    debug_port = rospy.get_param('~debug_port', 10000)
    user_agent = rospy.get_param(
        '~user_agent', 'Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; '
        'en-us AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
        'Version/4.0.4 Mobile/7B314 Safari/531.21.10'
    )
    state = rospy.get_param('~state', ApplicationState.VISIBLE)

    browser = ManagedBrowser(
        geometry=geometry,
        url=url,
        force_device_scale_factor=scale_factor,
        debug_port=debug_port,
        user_agent=user_agent
    )

    browser.set_state(state)

    rospy.Subscriber('{}/state'.format(rospy.get_name()), ApplicationState,
                     browser.handle_state_msg)

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('{}/debug'.format(rospy.get_name()), String, handle_debug_sock_msg)

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
