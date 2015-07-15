#! /usr/bin/env python

import rospy

from lg_common import ManagedWindow, ManagedBrowser
from lg_common.msg import ApplicationState

DEFAULT_URL = 'http://localhost:8008/lg_sv/webapps/client/index.html'


def main():
    rospy.init_node('streetview_browser', anonymous=True)
    geometry = ManagedWindow.get_viewport_geometry()
    url = str(rospy.get_param('~url', DEFAULT_URL))
    yawOffset = float(rospy.get_param('~yaw_offset', 0))
    pitchOffset = float(rospy.get_param('~pitch_offset', 0))
    showLinks = bool(rospy.get_param('~show_links', False))
    # put parameters into one big url
    url = "%s?yawOffset=%s&pitchOffset=%s&showLinks=%s" % (url, yawOffset, pitchOffset, str(showLinks).lower())
    # create the managed browser
    managed_browser = ManagedBrowser(url=url, geometry=geometry)

    # set to visible
    state = ApplicationState.VISIBLE
    managed_browser.set_state(state)

    # listen to state messages
    rospy.Subscriber('/streetview/state', ApplicationState, managed_browser.handle_state_msg)

    rospy.spin()

if __name__ == '__main__':
    main()
