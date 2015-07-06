#!/usr/bin/env python
"""
TODO
"""

from lg_common import ManagedBrowser
from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState
import rospy
from lg_adhoc_browser.msg import AdhocBrowser, AdhocBrowsers


class ManagedAdhocBrowser(ManagedBrowser):

    def __init__(self, geometry, slug, url):
        super(ManagedAdhocBrowser, self).__init__(
                geometry=geometry,
                slug=slug,
                url=url,
                app=True)

    def change_url(self):
        #TODO
        pass

    def close(self):
        self.set_state(ApplicationState.STOPPED)

# Handlers to all opened browser
# (id => ManagedAdhocBrowser)
browsers = {}


def message_handler(data):
    """
    
    data: AdhocBrowsers, which is an array of AdhocBrowser

    If the message has no AdhocBrowser messages in the array,
    close everything down.

    If the message has AdhocBrowser messages in the array:

    Check for an existing browser with the same id.
    If it exists, update the url and geometry.
    
    If no existing browser, create one.
    
    If an existing browser has an id that doesn't match the new message,
    shut it down and remove it from the tracked instances.
    
    """
    if not data.browsers: # close everything
        [b.close() for _, b in browsers.items()]

    for d in data.browsers:
        url = d.url
        id = d.id
        geometry = WindowGeometry(x=d.geometry.x,
                                  y=d.geometry.y,
                                  width=d.geometry.width, 
                                  height=d.geometry.height)
        b = browsers.get(id, None)
        if b:
            # change url
            pass
        else:
            b = ManagedAdhocBrowser(geometry=geometry, slug=id, url=url)
            b.set_state(ApplicationState.VISIBLE)
        browsers[id] = b


if __name__ == "__main__":
    rospy.init_node('lg_adhoc_browser')

    vieport_name = rospy.get_param('~viewport', None)
    if not vieport_name:
        rospy.logerr("Viewport is not set in the roslaunch file. Exiting.")
        exit(1)

    topic_name = '/browser_service/{}'.format(vieport_name)
    rospy.Subscriber(topic_name, AdhocBrowsers, message_handler) 

    rospy.spin()
