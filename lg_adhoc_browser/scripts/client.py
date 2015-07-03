#!/usr/bin/env python
"""
TODO
"""

from lg_common import ManagedBrowser
from lg_common.msg import WindowGeometry
from lg_common.msg import ApplicationState
import rospy


class ManagedAdhocBrowser(ManagedBrowser):

    def __init__(self, geometry, slug, url):
        super(ManagedAdhocBrowser, self).__init__(
                geometry=geometry,
                slug=slug,
                url=url,
                app=True)


if __name__ == "__main__":
    rospy.init_node('lg_adhoc_browser')

    url = "http://endpoint.com"
    slug = "slug"
    (x, y, w, h) = (50, 50, 300, 300)
    geometry = WindowGeometry(x=x, y=y, width=w, height=h)
    #rospy.loginfo(geometry)
    #m = ManagedBrowser(url=url, geometry=geometry, slug=slug, app=True)
    #rospy.loginfo(m)
    m = ManagedAdhocBrowser(geometry=geometry, slug=slug, url=url)
    m.set_state(ApplicationState.VISIBLE)

    rospy.spin()
