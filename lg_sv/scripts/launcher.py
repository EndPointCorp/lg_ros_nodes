#! /usr/bin/env python3

import rospy

from lg_common import ManagedWindow, ManagedBrowser, ManagedAdhocBrowser
from lg_msg_defs.msg import ApplicationState
from lg_common.helpers import add_url_params
from lg_common.helpers import check_www_dependency
from lg_common.helpers import discover_port_from_url, discover_host_from_url, x_available_or_raise
from lg_common.helpers import make_soft_relaunch_callback
from lg_common.helpers import run_with_influx_exception_handler
from lg_common.helpers import combine_viewport_geometries


DEFAULT_URL = 'http://localhost:8008/lg_sv/webapps/client/index.html'
# FOV for zoom level 3
DEFAULT_FOV = 28.125
NODE_NAME = 'panoviewer_browser'


def main():
    rospy.init_node(NODE_NAME, anonymous=True)

    viewports = rospy.get_param('~viewports', None)
    if viewports is None:
        geometry = ManagedWindow.get_viewport_geometry()
    else:
        viewports = [x.strip() for x in viewports.split(',')]
        geometry = combine_viewport_geometries(viewports)
    server_type = rospy.get_param('~server_type', 'streetview')
    url = str(rospy.get_param('~url', DEFAULT_URL))
    field_of_view = float(rospy.get_param('~fov', DEFAULT_FOV))
    pitch_offset = float(rospy.get_param('~pitch_offset', 0))
    show_links = str(rospy.get_param('~show_links', False)).lower()
    show_fps = str(rospy.get_param('~show_fps', False)).lower()
    show_attribution = str(rospy.get_param('~show_attribution', False)).lower()
    yaw_offset = float(rospy.get_param('~yaw_offset', 0))
    yaw_offsets = str(rospy.get_param('~yaw_offsets', yaw_offset))
    leader = str(rospy.get_param('~leader', 'false'))
    tilt = str(rospy.get_param('~tilt', 'false'))
    depend_on_webserver = rospy.get_param('~depend_on_webserver', False)
    depend_on_rosbridge = rospy.get_param('~depend_on_rosbridge', False)
    rosbridge_host = rospy.get_param('~rosbridge_host', '127.0.0.1')
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    rosbridge_secure = rospy.get_param('~rosbridge_secure', 'false')
    zoom = str(rospy.get_param('~zoom', 'false')).lower()
    initial_zoom = rospy.get_param('~initial_zoom', 3)
    kiosk = rospy.get_param('~kiosk', True)
    map_api_key = rospy.get_param('/google/maps_api_key', None)

    # put parameters into one big url
    url = add_url_params(url,
                         zoom=zoom,
                         initialZoom=initial_zoom,
                         fov=field_of_view,
                         pitchOffset=pitch_offset,
                         showLinks=show_links,
                         showFPS=show_fps,
                         showAttribution=show_attribution,
                         leader=leader,
                         yawOffset=yaw_offset,
                         yawOffsets=yaw_offsets,
                         tilt=tilt,
                         rosbridgeHost=rosbridge_host,
                         rosbridgePort=rosbridge_port,
                         rosbridgeSecure=rosbridge_secure)

    rospy.loginfo('Use Google maps API key: {}'.format(map_api_key))

    if map_api_key:
        url = add_url_params(url, map_api_key=map_api_key)

    # check if server is already there
    host = discover_host_from_url(url)
    port = discover_port_from_url(url)
    timeout = rospy.get_param('/global_dependency_timeout', 15)

    check_www_dependency(depend_on_webserver, host, port, 'streetview server', timeout)
    check_www_dependency(depend_on_webserver, rosbridge_host, rosbridge_port, 'rosbridge', timeout)

    x_available_or_raise(timeout)

    # create the managed browser

    slug = (server_type + "__" + "fov-" + str(field_of_view) + "__" + "yaw-" +
            str(yaw_offset) + "__" + "pitch-" + str(pitch_offset) +
            "__" + str(viewports[0]))
    managed_browser = ManagedAdhocBrowser(url=url, geometry=geometry, slug=slug, kiosk=kiosk)

    # set initial state
    state = ApplicationState.STOPPED
    managed_browser.set_state(state)

    def state_proxy(msg):
        if not msg.state == ApplicationState.VISIBLE:
            managed_browser.set_state(ApplicationState.HIDDEN)
        else:
            managed_browser.set_state(ApplicationState.VISIBLE)

    # listen to state messages
    rospy.Subscriber('/%s/state' % server_type, ApplicationState, state_proxy)
    make_soft_relaunch_callback(managed_browser.handle_soft_relaunch, groups=['streetview'])

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
