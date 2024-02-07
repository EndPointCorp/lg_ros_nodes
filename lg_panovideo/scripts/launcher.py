#! /usr/bin/env python3

import rospy

from lg_common import ManagedWindow, ManagedBrowser, ManagedAdhocBrowser
from lg_msg_defs.msg import ApplicationState, WindowGeometry
from lg_common.helpers import add_url_params
from lg_common.helpers import check_www_dependency
from lg_common.helpers import discover_port_from_url, discover_host_from_url, x_available_or_raise
from lg_common.helpers import make_soft_relaunch_callback
from lg_common.helpers import run_with_influx_exception_handler
from lg_common.helpers import has_activity, on_new_scene
from lg_common.helpers import combine_viewport_geometries


DEFAULT_URL = 'http://localhost:8008/lg_panovideo/webapps/panovideosync/index.html'
DEFAULT_FOV = 30
NODE_NAME = 'panovideo_browser'


def main():
    rospy.init_node(NODE_NAME)

    viewports = str(rospy.get_param('~viewports'))
    viewports = [x.strip() for x in viewports.split(',')]
    geometry = combine_viewport_geometries(viewports)
    url = str(rospy.get_param('~url', DEFAULT_URL))
    field_of_view = float(rospy.get_param('~fov', DEFAULT_FOV))
    yaw_offset = float(rospy.get_param('~yaw_offset', 0))
    yaw_offsets = str(rospy.get_param('~yaw_offsets', yaw_offset))
    leader = str(rospy.get_param('~leader', 'false'))
    clock_addr = rospy.get_param('~clock_addr', 'ws://localhost:9091')
    depend_on_webserver = rospy.get_param('~depend_on_webserver', False)
    depend_on_rosbridge = rospy.get_param('~depend_on_rosbridge', False)
    rosbridge_host = rospy.get_param('~rosbridge_host', 'localhost')
    rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
    rosbridge_secure = rospy.get_param('~rosbridge_secure', 'false')
    kiosk = rospy.get_param('~kiosk', True)

    url = add_url_params(url,
                         hFov=field_of_view,
                         master=leader,
                         clockAddr=clock_addr,
                         yawOffsets=yaw_offsets,
                         rosbridgeHost=rosbridge_host,
                         rosbridgePort=rosbridge_port,
                         rosbridgeSecure=rosbridge_secure)

    host = discover_host_from_url(url)
    port = discover_port_from_url(url)
    timeout = rospy.get_param('/global_dependency_timeout', 15)

    check_www_dependency(depend_on_webserver, host, port, 'panovideo server', timeout)
    check_www_dependency(depend_on_webserver, rosbridge_host, rosbridge_port, 'rosbridge', timeout)

    x_available_or_raise(timeout)

    slug = "lg_panovideo__" + str(geometry).replace('\n', '_').replace(': ', '_')
    managed_browser = ManagedAdhocBrowser(
        url=url,
        geometry=geometry,
        slug=slug,
        kiosk=kiosk,
        layer=ManagedWindow.LAYER_NORMAL,
    )

    # set initial state
    state = ApplicationState.STOPPED
    managed_browser.set_state(state)

    make_soft_relaunch_callback(managed_browser.handle_soft_relaunch, groups=['panovideo'])

    def handle_scene(scene):
        has_panovideo = has_activity(scene, 'panovideo')
        if has_panovideo:
            managed_browser.set_state(ApplicationState.VISIBLE)
        else:
            managed_browser.set_state(ApplicationState.STOPPED)

    on_new_scene(handle_scene)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
