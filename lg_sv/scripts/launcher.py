#! /usr/bin/env python

import rospy

from lg_common import ManagedWindow, ManagedBrowser, ManagedAdhocBrowser
from lg_common.msg import ApplicationState
from lg_common.helpers import add_url_params
from lg_common.helpers import dependency_available
from lg_common.helpers import discover_port_from_url, discover_host_from_url, x_available
from lg_common.helpers import DependencyException

DEFAULT_URL = 'http://localhost:8008/lg_sv/webapps/client/index.html'
# FOV for zoom level 3
DEFAULT_FOV = 28.125


def main():
    rospy.init_node('panoviewer_browser', anonymous=True)

    geometry = ManagedWindow.get_viewport_geometry()
    server_type = rospy.get_param('~server_type', 'streetview')
    url = str(rospy.get_param('~url', DEFAULT_URL))
    field_of_view = float(rospy.get_param('~fov', DEFAULT_FOV))
    pitch_offset = float(rospy.get_param('~pitch_offset', 0))
    show_links = str(rospy.get_param('~show_links', False)).lower()
    show_attribution = str(rospy.get_param('~show_attribution', False)).lower()
    yaw_offset = float(rospy.get_param('~yaw_offset', 0))
    leader = str(rospy.get_param('~leader', 'false'))
    tilt = str(rospy.get_param('~tilt', 'false'))
    depend_on_webserver = rospy.get_param('~depend_on_webserver', False)
    depend_on_rosbridge = rospy.get_param('~depend_on_rosbridge', False)
    rosbridge_host = rospy.get_param('~rosbridge_host', '127.0.0.1')
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    rosbridge_secure = rospy.get_param('~rosbridge_secure', 'false')
    zoom = str(rospy.get_param('~zoom', 'false')).lower()
    initial_zoom = rospy.get_param('~initial_zoom', 3)

    # put parameters into one big url
    url = add_url_params(url,
                         zoom=zoom,
                         initialZoom=initial_zoom,
                         fov=field_of_view,
                         pitchOffset=pitch_offset,
                         showLinks=show_links,
                         showAttribution=show_attribution,
                         leader=leader,
                         yawOffset=yaw_offset,
                         tilt=tilt,
                         rosbridgeHost=rosbridge_host,
                         rosbridgePort=rosbridge_port,
                         rosbridgeSecure=rosbridge_secure)

    # check if server is already there
    host = discover_host_from_url(url)
    port = discover_port_from_url(url)
    timeout = rospy.get_param('/global_dependency_timeout', 15)

    if depend_on_webserver:
        rospy.loginfo("Waiting for webserver to become available")
        if not dependency_available(host, port, 'streetview_server', timeout):
            msg = "Streetview server (%s:%s) did not appear within specified timeout of %s seconds" % (host, port, timeout)
            rospy.logerr(msg)
            raise DependencyException
        else:
            rospy.loginfo("Webserver available - continuing initialization")

    if depend_on_rosbridge:
        if not dependency_available(rosbridge_host, rosbridge_port, 'rosbridge', timeout):
            msg = "Rosbridge (%s:%s) did not appear within specified timeout of %s seconds" % (rosbridge_host, rosbridge_port, timeout)
            rospy.logerr(msg)
            raise DependencyException
        else:
            rospy.loginfo("ROSbridge available - continuing initialization")

    # wait for X to become available
    x_timeout = rospy.get_param("/global_dependency_timeout", 15)
    if x_available(x_timeout):
        rospy.loginfo("X available")
    else:
        msg = "X server is not available"
        rospy.logfatal(msg)
        raise DependencyException(msg)

    # create the managed browser
    slug = server_type + "__" +\
            "_fov-" + str(field_of_view) + "__" +\
            "_yaw-" + str(yaw_offset) + "__" + \
            "_pitch-" + str(pitch_offset)
    managed_browser = ManagedAdhocBrowser(url=url, geometry=geometry, slug=slug)

    # set to visible
    state = ApplicationState.HIDDEN
    managed_browser.set_state(state)

    # listen to state messages
    rospy.Subscriber('/%s/state' % server_type, ApplicationState, managed_browser.handle_state_msg)

    rospy.spin()

if __name__ == '__main__':
    main()
