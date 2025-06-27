#!/usr/bin/env python3

import rospy
from lg_msg_defs.msg import AdhocBrowsers, AdhocBrowser
from lg_common import AdhocBrowserPool
from lg_msg_defs.msg import AdhocMedias
from lg_common.helpers import add_url_params, make_soft_relaunch_callback
from urllib.request import url2pathname
from lg_common.helpers import run_with_influx_exception_handler

VIDEOSYNC_URL = 'http://localhost:8008/lg_media/webapps/videosync/index.html'
NODE_NAME = 'lg_media_browser_launcher'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


class BasicBrowserData:
    def __init__(self, publisher, leader, ros_port, ros_host, url, sync_rate,
                 frame_latency, ping_interval, hard_sync_diff,
                 min_playbackrate, max_playbackrate, autoplay, show_controls,
                 viewport_name):
        self.publisher = publisher
        self.leader = leader
        self.show_controls = show_controls
        self.autoplay = autoplay
        self.ros_port = ros_port
        self.ros_host = ros_host
        self.url = url
        self.sync_rate = sync_rate
        self.frame_latency = frame_latency
        self.ping_interval = ping_interval
        self.hard_sync_diff = hard_sync_diff
        self.min_playbackrate = min_playbackrate
        self.max_playbackrate = max_playbackrate
        self.viewport_name = viewport_name

    def launch_browser(self, data):
        """
        data: AdhocMedias, which is a list of AdhocMedia objects

        Turns these medias into AdhocBrowsers and then publishes them
        """
        msg = AdhocBrowsers()
        for media in data.medias:
            url = add_url_params(
                self.url,
                videoUrl=media.url,
                master=self.leader,
                loop=media.loop,
                sync=True,
            )
            url = url2pathname(url)
            logger.debug('url for media: %s' % url)
            new_browser = AdhocBrowser()
            new_browser.id = 'adhoc_media_browser_%s' % self.viewport_name
            new_browser.geometry = media.geometry
            new_browser.url = url
            msg.browsers.append(new_browser)
            logger.debug("New browser URL: %s" % url)

        self.publisher.publish(msg)


def main():
    rospy.init_node(NODE_NAME)

    viewport_name = rospy.get_param('~viewport', None)
    if not viewport_name:
        msg = "Viewport not configured for lg_media browser_launcher - exiting"
        logger.error(msg)
        exit(1)

    browser_pool_publisher = rospy.Publisher('/media_service/launch_browser/%s' % viewport_name,
                                             AdhocBrowsers, queue_size=10)
    is_leader = str(rospy.get_param('~leader', False)).lower()
    ros_port = str(rospy.get_param('~ros_port', '9090'))
    ros_host = str(rospy.get_param('~ros_host', 'localhost'))
    url = str(rospy.get_param('~videosync_url', VIDEOSYNC_URL))
    sync_rate = str(rospy.get_param('~sync_rate', 60))
    frame_latency = str(rospy.get_param('~frame_latency', 3 / 25))
    ping_interval = str(rospy.get_param('~ping_interval', 1000))
    hard_sync_diff = str(rospy.get_param('~hard_sync_diff', 1.0))
    min_playbackrate = str(rospy.get_param('~min_playbackrate', 0.5))
    max_playbackrate = str(rospy.get_param('~max_playbackrate', 1.5))
    autoplay = str(rospy.get_param('~autoplay', False)).lower()
    show_controls = str(rospy.get_param('~show_controls', False)).lower()

    basic_browser_data = BasicBrowserData(browser_pool_publisher, is_leader,
                                          ros_port, ros_host, url, sync_rate,
                                          frame_latency, ping_interval,
                                          hard_sync_diff, min_playbackrate,
                                          max_playbackrate, autoplay,
                                          show_controls, viewport_name)

    browser_pool = AdhocBrowserPool(viewport_name)
    make_soft_relaunch_callback(browser_pool.handle_soft_relaunch, groups=["media"])

    rospy.Subscriber('/media_service/browser/%s' % viewport_name, AdhocMedias,
                     basic_browser_data.launch_browser)

    rospy.Subscriber('/media_service/launch_browser/%s' % viewport_name, AdhocBrowsers,
                     browser_pool.handle_ros_message)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
