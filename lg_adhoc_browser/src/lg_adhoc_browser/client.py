#!/usr/bin/env python
"""
TODO
"""

from lg_common import ManagedBrowser

class Client(ManagedBrowser):

    def __init__(self):
        super(Client, this)


from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState

if __name__ == "__main__":
    rospy.init_node(rospy.get_param('~viewport', ''))

    url = "endpoint.com"
    slug = "slug"
    x = 10
    y = 20
    w = 1000
    h = 500
    geometry = WindowGeometry(x=x, y=y, width=w, height=h)
    
    m = ManagedBrowser(url=url, geometry=geometry, slug=slug)
    m.set_state(ApplicationState.Visible)

