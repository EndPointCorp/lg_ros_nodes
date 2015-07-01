#!/usr/bin/env python
"""
TODO
"""

from lg_common import ManagedBrowser
from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState
import rospy

if __name__ == "__main__":
    rospy.init_node('lg_adhoc_browser')

    url = "http://endpoint.com"
    slug = "slug"
    x = 10
    y = 20
    w = 1000
    h = 500
    geometry = WindowGeometry(x=x, y=y, width=w, height=h)
    
    m = ManagedBrowser(url=url, geometry=geometry, slug=slug)
    m.set_state(ApplicationState.VISIBLE)
    rospy.spin()
    #m.set_state(ApplicationState.Visible)

