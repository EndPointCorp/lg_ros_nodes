#!/usr/bin/env python

import rospy


from urllib import url2pathname
from std_msgs.msg import String
from lg_common.helpers import add_url_params
from lg_common import ManagedBrowser, ManagedWindow
from lg_common.msg import ApplicationState, WindowGeometry
from lg_common.helpers import dependency_available, x_available
from lg_common.helpers import DependencyException
from lg_common.helpers import x_available


if __name__ == '__main__':
    rospy.init_node('static_browser')

    geometry = ManagedWindow.get_viewport_geometry()

    url_base = rospy.get_param('~url_base', 'http://lg-head/ros_touchscreens/ts/')
    command_line_args = rospy.get_param('~command_line_args', '')
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

    if depend_on_rosbridge:
        rospy.loginfo("Waiting for rosbridge to become available")
        if not dependency_available(rosbridge_host, rosbridge_port, 'rosbridge', global_dependency_timeout):
            msg = "Service: %s hasn't become accessible within %s seconds" % ('rosbridge', global_dependency_timeout)
            rospy.logfatal(msg)
            raise DependencyException(msg)
        else:
            rospy.loginfo("Rosbridge is online")

    if depend_on_director:
        rospy.loginfo("Waiting for director to become available")
        if not dependency_available(director_host, director_port, 'director', global_dependency_timeout):
            msg = "Service: %s hasn't become accessible within %s seconds" % ('director', global_dependency_timeout)
            rospy.logfatal(msg)
            raise DependencyException(msg)
        else:
            rospy.loginfo("Director is online")

    x_timeout = rospy.get_param("/global_dependency_timeout", 15)
    if x_available(x_timeout):
        rospy.loginfo("X available")
    else:
        msg = "X server is not available"
        rospy.logfatal(msg)
        raise DependencyException(msg)

    url = url_base + ts_name + "/"

    url = add_url_params(url,
                         director_host=director_host,
                         director_port=director_port,
                         director_secure=director_secure,
                         rosbridge_secure=rosbridge_secure,
                         rosbridge_host=rosbridge_host,
                         rosbridge_port=rosbridge_port)

    url = url2pathname(url)

    rospy.loginfo("got url: %s" % url)

    scale_factor = rospy.get_param('~force_device_scale_factor', 1)
    debug_port = rospy.get_param('~debug_port', 10000)
    user_agent = rospy.get_param(
        '~user_agent', 'Mozilla/5.0(iPad; U; CPU iPhone OS 3_2 like Mac OS X; '
        'en-us AppleWebKit/531.21.10 (KHTML, like Gecko) ' +
        'Version/4.0.4 Mobile/7B314 Safari/531.21.10'
    )

    browser = ManagedBrowser(
        geometry=geometry,
        url=url,
        command_line_args=command_line_args,
        force_device_scale_factor=scale_factor,
        debug_port=debug_port,
        user_agent=user_agent
    )

    browser.set_state(ApplicationState.VISIBLE)

    def handle_debug_sock_msg(msg):
        browser.send_debug_sock_msg(msg.data)

    rospy.Subscriber('{}/debug'.format(rospy.get_name()), String, handle_debug_sock_msg)

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
