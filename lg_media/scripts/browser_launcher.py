#!/usr/bin/env python

import rospy
from lg_common.msg import AdhocBrowsers, AdhocBrowser
from lg_common import AdhocBrowserPool
from lg_media.msg import AdhocMedias
from lg_common.helpers import add_url_params
from urllib import url2pathname

VIDEOSYNC_URL = 'http://localhost:8008/lg_sv/webapps/videosync/index.html'


class BasicBrowserData:
    def __init__(self, publisher, leader, ros_port, ros_host, url, sync_rate,
                 frame_latency, ping_interval, hard_sync_diff,
                 min_playbackrate, max_playbackrate, autoplay, show_controls):
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

    def launch_browser(self, data):
        """
        data: AdhocMedias, which is a list of AdhocMedia objects

        Turns these medias into AdhocBrowsers and then publishes them
        """
        msg = []
        for media in data.medias:
            url = add_url_params(
                self.url, src=media.url,
                leader=self.leader,
                autoplay=self.autoplay,
                show_controls=self.show_controls,
                rosbridge_port=self.ros_port,
                rosbridge_host=self.ros_host,
                syncRate=self.sync_rate,
                frameLatency=self.frame_latency,
                pingInterval=self.ping_interval,
                hardSyncDiff=self.hard_sync_diff,
                minPlaybackRate=self.min_playbackrate,
                maxPlaybackRate=self.max_playbackrate)
            url = url2pathname(url)
            rospy.logerr('url for media: %s' % url)
            new_browser = AdhocBrowser()
            new_browser.geometry = media.geometry
            new_browser.url = url
            msg.append(new_browser)

        self.publisher.publish(msg)


def main():
    rospy.init_node('browser_launcher')

    viewport_name = rospy.get_param('~viewport', None)
    if not viewport_name:
        msg = "Viewport not configured for lg_media browser_launcher - exiting"
        rospy.logerr(msg)
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
                                          show_controls)

    browser_pool = AdhocBrowserPool(viewport_name)

    rospy.Subscriber('/media_service/launch_browser/%s' % viewport_name, AdhocBrowsers,
                     browser_pool.handle_ros_message)
    rospy.Subscriber('/media_service/browser/%s' % viewport_name, AdhocMedias,
                     basic_browser_data.launch_browser)

    rospy.spin()

if __name__ == '__main__':
    main()
