#!/usr/bin/env python3

import json
from collections import namedtuple

import rospy
from lg_common import ManagedWindow
from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.msg import MediaOverlays

StreamInfo = namedtuple('overlay', ['name', 'viewport', 'location', 'url'])

class MediaLauncher(object):
    """Maintain a list of active streams based on MediaOverlays messages"""
    def __init__(self, janus_host, janus_port, pub, viewports):
        self.director_pub = pub
        self.janus_host = janus_host
        self.janus_port = janus_port
        self.viewports = viewports

        self.active_overlays = {}

    def handle_message(self, message):
        """Handle incoming ros messages
        Converts MediaOverlays message to list of named tuples
        Set diff against active overlays to determine streams to add/remove
        """
        message_overlays = []
        for msg in message.overlays:
            message_overlays.append(StreamInfo(msg.name, msg.viewport, msg.location, self._make_url(msg.location)))

        stale_overlays = set(self.active_overlays.keys()).difference(set(message_overlays))
        new_overlays = set(message_overlays).difference(set(self.active_overlays.keys()))

        rospy.loginfo(
            f"New message. Media in the msg: {len(message_overlays)} | "
            f"Overlays to remove: {len(stale_overlays)} | "
            f"Overlays to add: {len(new_overlays)}"
        )
        rospy.logdebug(f"Stale overlays \n\t{stale_overlays}\nNew overlays\n\t{new_overlays}")

        for item in stale_overlays:
            rospy.logdebug(f"Removing overlay from active \n\t{item}")
            if self.active_overlays.pop(item, None) is None:
                rospy.logerr(f"Error removing expected active overlay: {item}")

        for item in new_overlays:
            rospy.logdebug(f"Adding overlay to active \n\t{item}")
            self.active_overlays[item] = True

        self.director_pub.publish(make_director_message(self.active_overlays.keys()))

        rospy.loginfo(
            f"Finshed message handling. | "
            f"Active overlays: {len(self.active_overlays)}"
        )

    def _make_url(self, stream_id):
        return(
            f"http://localhost:8008/lg_media/webapps/janusplayer/index.html?"
            f"janusHost={self.janus_host}&janusPort={self.janus_port}&streamID={stream_id}"
        )


def make_director_message(overlays):
    browser_windows = []
    msg = GenericMessage()
    msg.type = "json"

    for overlay in overlays:
        geometry = ManagedWindow.lookup_viewport_geometry(overlay.viewport)
        browser_windows.append(
            {
                "activity": "browser",
                "activity_config": {},
                "assets": [overlay.url],
                "height": geometry.height,
                "presentation_viewport": overlay.viewport,
                "width": geometry.width,
                "x_coord": 0,
                "y_coord": 0
            }
        )

    director_msg = {
        "description": "testdescription",
        "duration": 0,
        "name": "Media Overlays",
        "resource_uri": "/this/isnt/real",
        "slug": "current_overlays",
        "windows": browser_windows
    }

    msg.message = json.dumps(director_msg)

    return msg

def main():
    rospy.init_node('media_launcher', log_level=rospy.INFO)

    janus_host = rospy.get_param('~janus_host', 'localhost')
    janus_port = rospy.get_param('~janus_port', 8088)

    viewports = [param.strip() for param in rospy.get_param('~viewports', '').split(',')]
    overlay_director_pub = rospy.Publisher('director/scene', GenericMessage, queue_size=10)
    launcher = MediaLauncher(janus_host, janus_port, overlay_director_pub, viewports)

    rospy.Subscriber('/media_overlays', MediaOverlays, launcher.handle_message)

    rospy.loginfo(f"Started media_launcher node on topic /media_overlays for viewports {viewports}")

    rospy.spin()

if __name__ == '__main__':
    main()
