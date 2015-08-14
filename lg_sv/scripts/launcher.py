#! /usr/bin/env python

import rospy

from lg_common import ManagedWindow, ManagedBrowser
from lg_common.msg import ApplicationState
from lg_common.helpers import add_url_params

DEFAULT_URL = 'http://localhost:8008/lg_sv/webapps/client/index.html'
#FOV for zoom level 3
DEFAULT_FOV = 28.125


def main():
    rospy.init_node('panoviewer_browser', anonymous=True)
    geometry = ManagedWindow.get_viewport_geometry()
    server_type = rospy.get_param('~server_type', 'streetview')
    url = str(rospy.get_param('~url', DEFAULT_URL))
    field_of_view = float(rospy.get_param('~fov', DEFAULT_FOV))
    pitch_offset = float(rospy.get_param('~pitch_offset', 0))
    show_links = str(rospy.get_param('~show_links', False)).lower()
    yaw_offset = float(rospy.get_param('~yaw_offset', 0))
    leader = str(rospy.get_param('~leader', 'false'))
    # put parameters into one big url
    url = add_url_params(url,
                         fov=field_of_view,
                         pitchOffset=pitch_offset,
                         showLinks=show_links,
                         leader=leader,
                         yawOffset=yaw_offset)
    # create the managed browser
    managed_browser = ManagedBrowser(url=url, geometry=geometry)

    # set to visible
    state = ApplicationState.VISIBLE
    managed_browser.set_state(state)

    # listen to state messages
    rospy.Subscriber('/%s/state' % server_type, ApplicationState, managed_browser.handle_state_msg)

    rospy.spin()

if __name__ == '__main__':
    main()
