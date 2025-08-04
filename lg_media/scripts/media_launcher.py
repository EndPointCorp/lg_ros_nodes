#!/usr/bin/env python3

from collections import namedtuple

import rospy
from lg_common import ManagedWindow, ManagedBrowser
from lg_msg_defs.msg import ApplicationState, MediaOverlays
from lg_common.logger import get_logger
logger = get_logger('media_launcher')

"""media_launcher.py
Receives ros messages with stream and viewport name and starts ManagedBrowsers
Intended to manage persistent windows outside Director control
"""

StreamInfo = namedtuple('overlay', ['name', 'viewport'])

# Dict to store ManagedBrowser objects by StreamInfo tuple
active_overlays = {}


def handle_message(message: MediaOverlays) -> None:
    """Handle incoming ros messages and starts/stops browsers accordingly"""
    message_overlays = []

    for msg in message.overlays:
        if msg.viewport in viewports:
            message_overlays.append(StreamInfo(msg.name, msg.viewport))
        else:
            logger.error(f"MediaOverlay specifies undefined viewport: {msg}")

    stale_overlays = set(active_overlays.keys()).difference(set(message_overlays))
    new_overlays = set(message_overlays).difference(set(active_overlays.keys()))

    logger.debug(
        f"New MediaOverlays definition: {message_overlays} | "
    )

    for item in stale_overlays:
        logger.debug(f"Removing media overlay browser \n\t{item}")
        stale_overlay = active_overlays.pop(item)
        stale_overlay.set_state(ApplicationState.STOPPED)

    for item in new_overlays:
        logger.debug(f"Adding media overlay browser \n\t{item}")
        active_overlays[item] = ManagedBrowser(
            url=f"{url_base}?name={item.name}",
            slug=f"media_launcher_overlay_{'_'.join(item)}",
            geometry=ManagedWindow.lookup_viewport_geometry(item.viewport),
            layer=ManagedWindow.LAYER_ABOVE,
        )
        active_overlays[item].set_state(ApplicationState.VISIBLE)


def main():
    global url_base
    global viewports
    rospy.init_node('media_launcher', log_level=rospy.INFO)
    logger.debug(f"Starting media_launcher node")
    url_base = rospy.get_param(
        param_name="~url_base",
        default="http://lg-head/touchscreen_assets/streams/"
    )
    logger.info(f"Browsers will use URL {url_base}?name=MediaOverlay.name")

    if rospy.has_param("~viewports"):
        viewports = [v.strip() for v in rospy.get_param("~viewports").strip().split(",")]
    else:
        viewports = [k.strip() for k in rospy.get_param_names() if k.startswith("/viewport/")]
    logger.debug(f"configured viewports: {viewports}")

    rospy.Subscriber(
        name='/media_overlays',
        data_class=MediaOverlays,
        callback=handle_message,
    )

    rospy.spin()


if __name__ == '__main__':
    main()
