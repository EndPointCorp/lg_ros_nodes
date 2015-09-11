#!/usr/bin/env python

import rospy
from lg_common.msg import AdhocBrowsers, AdhocBrowser
from lg_common import AdhocBrowserPool
from lg_media.msg import AdhocMedias
from lg_common.helpers import add_url_params

DEFAULT_VIEWPORT = 'center'
VIDEOSYNC_URL = 'http://lg-head/lg_sv/webapps/videosync'


class BasicBrowserData:
    def __init__(self, publisher, leader, ros_port, ros_host, url):
        self.publisher = publisher
        self.leader = leader
        self.ros_port = ros_port
        self.ros_host = ros_host
        self.url = url


    def launch_browser(self, data):
        """
        data: AdhocMedias, which is a list of AdhocMedia objects

        Turns these medias into AdhocBrowsers and then publishes them
        """
        msg = []
        for media in data.medias:
            url = add_url_params(
                self.url, src=media.url, leader=self.leader,
                rosbridge_port=self.ros_port,
                rosbridge_host=self.ros_host)
            rospy.logerr('url for media: %s' % url)
            new_browser = AdhocBrowser()
            new_browser.geometry = media.geometry
            new_browser.url = url
            msg.append(new_browser)

        self.publisher.publish(msg)


def main():
    rospy.init_node('browser_launcher')

    browser_pool_publisher = rospy.Publisher('/media_service/launch_browser',
                                             AdhocBrowsers, queue_size=10)
    is_leader = str(rospy.get_param('~leader', False)).lower()
    ros_port = str(rospy.get_param('~ros_port', '9090'))
    ros_host = str(rospy.get_param('~ros_host', 'localhost'))
    url = str(rospy.get_param('~videosync_url', VIDEOSYNC_URL))
    basic_browser_data = BasicBrowserData(browser_pool_publisher, is_leader,
                                          ros_port, ros_host, url)

    viewport_name = rospy.get_param('~viewport', DEFAULT_VIEWPORT)
    browser_pool = AdhocBrowserPool(viewport_name)

    rospy.Subscriber('/media_service/launch_browser', AdhocBrowsers,
                     browser_pool.handle_ros_message)
    rospy.Subscriber('/media_service/%s' % viewport_name, AdhocMedias,
                     basic_browser_data.launch_browser)

    rospy.spin()

if __name__ == '__main__':
    main()
