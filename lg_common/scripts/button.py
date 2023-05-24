#!/usr/bin/env python3

import rospy

from urllib.request import url2pathname
from std_msgs.msg import String
from lg_common.helpers import add_url_params
from lg_common import ManagedBrowser, ManagedWindow, ManagedApplication, ManagedAdhocBrowser
from lg_msg_defs.msg import ApplicationState, WindowGeometry
from lg_common.helpers import x_available_or_raise, check_www_dependency
from lg_common.helpers import make_soft_relaunch_callback
from lg_common.helpers import run_with_influx_exception_handler


DEFAULT_URL = 'http://localhost:8008/lg_common/webapps/button.html'
NODE_NAME = 'static_browser'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)
state = None


def main():
    rospy.init_node(NODE_NAME)

    rospy.get_param('~viewport', 'touchscreen_button')
    url = rospy.get_param('~url', DEFAULT_URL)
    geometry = ManagedWindow.get_viewport_geometry()
    window = ManagedWindow(
        w_instance='unique_button',
        geometry=geometry,
        layer=ManagedWindow.LAYER_TOUCH,
    )
    logger.error(f"geometry is {geometry}")
    managed_browser = ManagedAdhocBrowser(
        url=url,
        geometry=geometry,
        layer=ManagedWindow.LAYER_TOUCH,
    )

    managed_browser.set_state(ApplicationState.VISIBLE)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
