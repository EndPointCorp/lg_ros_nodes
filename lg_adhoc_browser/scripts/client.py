#!/usr/bin/env python
"""
TODO: Browser is not changing url yet.
"""

import rospy
from lg_adhoc_browser.msg import AdhocBrowser, AdhocBrowsers
from lg_adhoc_browser.managed_browser import ManagedAdhocBrowser
from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState


"""
Handlers to all opened browsers
Dict(id => ManagedAdhocBrowser)
"""
browsers = {}


def message_handler(data):
    """
    This function handles the ros messages, and manages the browsers.
    
    If the message has no AdhocBrowser messages in the array,
    close everything down.

    If the message has AdhocBrowser messages in the array:

    Check for an existing browser with the same id.
    If it exists, update the url and geometry.
    
    If no existing browser, create one.
    
    If an existing browser has an id that doesn't match the new message,
    shut it down and remove it from the tracked instances.

    Arguments:
        data: AdhocBrowsers, which is an array of AdhocBrowser
    
    """
    # Let's repack the data from the ros message, it will be easier to use it later in this for.
    # Dict(id => AdhocBrowser)
    new_browsers = {b.id:b for b in data.browsers}

    # And operate on sets of ids.
    current_browsers_ids = set(browsers.keys())
    new_browsers_ids = set(new_browsers.keys())

    # Create sets with ids to remove, create, and udpate.
    remove_ids = current_browsers_ids - new_browsers_ids
    create_ids = new_browsers_ids - current_browsers_ids
    update_ids = new_browsers_ids | current_browsers_ids

    # Close all browsers which are not on the new list, and remove them from the browsers dict.
    for id in remove_ids:
        browsers[id].close()
        del browsers[id]

    # Create new browsers, and add them to the browsers dict.
    for id in create_ids:
        d = new_browsers[id]
        url = d.url
        geometry = WindowGeometry(x=d.geometry.x,
                                  y=d.geometry.y,
                                  width=d.geometry.width, 
                                  height=d.geometry.height)
        b = ManagedAdhocBrowser(geometry=geometry, slug=id, url=url)
        b.set_state(ApplicationState.VISIBLE)
        browsers[id] = b

    # Update existing browsers.
    for id in update_ids:
        b = browsers[id]
        d = new_browsers[id]
            
        geometry = WindowGeometry(x=d.geometry.x,
                                  y=d.geometry.y,
                                  width=d.geometry.width, 
                                  height=d.geometry.height)
        b.update_geometry()
        b.update_url(url)


if __name__ == "__main__":
    rospy.init_node('lg_adhoc_browser')

    vieport_name = rospy.get_param('~viewport', None)
    if not vieport_name:
        rospy.logerr("Viewport is not set in the roslaunch file. Exiting.")
        exit(1)

    topic_name = '/browser_service/{}'.format(vieport_name)
    rospy.Subscriber(topic_name, AdhocBrowsers, message_handler) 

    rospy.spin()
