#!/usr/bin/env python

import rospy


from urllib import url2pathname
from std_msgs.msg import String
from lg_common.helpers import add_url_params
from lg_common import ManagedBrowser, ManagedWindow
from lg_common.msg import ApplicationState, WindowGeometry
from lg_common.helpers import x_available_or_raise, check_www_dependency
from lg_common.helpers import make_soft_relaunch_callback
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'static_browser'


def main():
    rospy.init_node(NODE_NAME)

    geometry = ManagedWindow.get_viewport_geometry()

    url = rospy.get_param('~url', None)
    url_base = rospy.get_param('~url_base', 'http://lg-head/ros_touchscreens/ts/')
    command_line_args = rospy.get_param('~command_line_args', '')
    extra_logging = rospy.get_param('~extra_logging', False)

    # TODO (wz) director_host and director_port should be global

    director_host = rospy.get_param('~director_host', '42-a')
    director_port = rospy.get_param('~director_port', 8060)

    rosbridge_secure = rospy.get_param('~rosbridge_secure', 0)
    director_secure = rospy.get_param('~director_secure', 0)

    rosbridge_host = rospy.get_param('~rosbridge_host', '127.0.0.1')
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    ts_name = rospy.get_param('~ts_name', 'default')

    depend_on_rosbridge = rospy.get_param('~depend_on_rosbridge', False)
    depend_on_director = rospy.get_param('~depend_on_director', False)
    global_dependency_timeout = rospy.get_param('/global_dependency_timeout', 15)

    check_www_dependency(depend_on_rosbridge, rosbridge_host, rosbridge_port, 'rosbridge', global_dependency_timeout)
    check_www_dependency(depend_on_director, director_host, director_port, 'director', global_dependency_timeout)

    x_available_or_raise(global_dependency_timeout)

    if url:
        rospy.loginfo("got prepared full url: %s" % url)
    else:
        url = url_base + ts_name + "/"

        url = add_url_params(url,
                             director_host=director_host,
                             director_port=director_port,
                             director_secure=director_secure,
                             rosbridge_secure=rosbridge_secure,
                             rosbridge_host=rosbridge_host,
                             rosbridge_port=rosbridge_port)

        url = url2pathname(url)
        rospy.loginfo("assembled a url: %s" % url)

    scale_factor = rospy.get_param('~force_device_scale_factor', 1)
    debug_port = rospy.get_param('~debug_port', None)
    user_agent = rospy.get_param(
        '~user_agent', 'Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; '
        'en-us AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
        'Version/4.0.4 Mobile/7B314 Safari/531.21.10'
    )
    log_level = rospy.get_param('/logging/level', 0)

    browser = ManagedBrowser(
        geometry=geometry,
        url=url,
        log_level=log_level,
        command_line_args=command_line_args,
        log_stderr=extra_logging,
        force_device_scale_factor=scale_factor,
        remote_debugging_port=debug_port,
        user_agent=user_agent
    )

    browser.set_state(ApplicationState.VISIBLE)
    make_soft_relaunch_callback(browser.handle_soft_relaunch, groups=['touchscreen'])

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('{}/debug'.format(rospy.get_name()), String, handle_debug_sock_msg)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
