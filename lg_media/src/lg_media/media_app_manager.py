#!/usr/bin/env python
"""
Adhoc media service.

define video playing commands (String),
http://www.mplayerhq.hu/DOCS/HTML/en/control.html
http://www.mplayerhq.hu/DOCS/tech/slave.txt
http://stackoverflow.com/questions/4976276/is-it-possible-to-control-mplayer-from-another-program-easily

notes:
    rospy.loginfo()
    while not rospy.is_shutdown() ...
    timeout = rospy.get_param('~timeout', 1)
    rospy.ROSInterruptException:
    rospy.service.ServiceException:


catkin package set up:
    catkin_create_pkg lg_media std_msgs rospy

catking_make
source catkin/devel/setup.bash
roslaunch lg_media/launch/dev.launch --screen
rostopic list
rostopic pub --once /media_service/test_viewport_1 std_msgs/String -- "aa bb cc"
rostopic pub --once /media_service/left_one lg_media/AdhocMedias '[{id: "1", url: /mnt/data/video/humour/kaiser_labus-7_statecnych.flv}]'
rostopic pub --once /media_service/left_one lg_media/AdhocMedias '[{id: "1", url: /mnt/data/video/humour/kaiser_labus-7_statecnych.flv, geometry: {x: 640, y: 480, width: 0, height: 0}}]'

# mplayer window is not placed on a desired geometry, investigate
local file URL works without quotes or with quotes:
rostopic pub --once /media_service/left_one lg_media/AdhocMedias '[{id: "1", url: /mnt/data/video/humour/kaiser_labus-7_statecnych.flv, geometry: {x: 640, y: 480, width: 0, height: 0}}]'
rostopic pub --once /media_service/left_one lg_media/AdhocMedias '[{id: "1", url: /mnt/data/video/humour/kaiser_labus-7_statecnych.flv, geometry: {x: 640, y: 480, width: 0, height: 0}},{id: "2", url: /mnt/data/video/humour/walker_texas_ranger.flv, geometry: {x: 0, y: 0, width: 0, height: 0}}]'
http URL has to have quotes, rostopic command parsing fails otherwise:
rostopic pub --once /media_service/left_one lg_media/AdhocMedias '[{id: "1", url: "https://zdenek.endpoint.com/kaiser_labus-7_statecnych.flv", geometry: {x: 640, y: 480, width: 0, height: 0}}]'

# shutdown test
rostopic pub --once /media_service/left_one lg_media/AdhocMedias '[]'

"""

import os

import rospy
from std_msgs.msg import String

from appctl_support import ProcController
from lg_common import ManagedApplication, ManagedWindow
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry


ROS_NODE_NAME = "lg_media"
DEFAULT_VIEWPORT = "test_viewport_0"
DEFAULT_APP = "mplayer"


class MediaService(object):
    """

    """

    @property
    def app_cmd(self):
        cmd = [rospy.get_param("~application_path", DEFAULT_APP)]
        cmd.extend(rospy.get_param("~application_flags").split())
        return cmd

    def __init__(self):
        self.apps = {}  # key: app id, value: app instance

    def listener(self, data):
        """
        Listener on a ROS topic.

        """
        rospy.logdebug("'listener' invoked, received data: '%s'" % data)
        if len(data.medias) > 0:
            curr_apps = {}
            for media in data.medias:
                if media.id in self.apps:
                    rospy.logdebug("App id '%s' exists, updating ..." % media.id)
                    app = self.apps[media.id]
                    # TODO
                    # update the url and geometry of the process app accordingly
                else:
                    rospy.logdebug("App id '%s' doesn't exist, starting ..." % media.id)
                    curr_apps[media.id] = self._start_and_get_app(media)
            self.apps.update(curr_apps)
        else:
            rospy.logdebug("Media array empty, shutting existing applications down ...")
            self._shutdown()

    def _start_and_get_app(self, media):
        """
        Start a ManagedApplication instance according to the details in the
        media argument and return instance.

        """
        # geometry = ManagedWindow.get_viewport_geometry()
        # "640x480+0+0" x, y, width, height
        # TODO
        # mplayer window is not placed on a desired geometry, investigate
        # -> should be up to awesome window manager
        geometry = WindowGeometry(x=media.geometry.x,
                                  y=media.geometry.y,
                                  width=media.geometry.width,
                                  height=media.geometry.height)
        #rospy.loginfo("geometry: '%s'" % geometry)
        # rospy.loginfo("instance: '%s'" % self._get_instance())
        mplayer_window = ManagedWindow(geometry=geometry,
                                       w_class="mplayer",
                                       w_name="mplayer",
                                       w_instance=self._get_instance())
        cmd = self.app_cmd
        cmd.extend([media.url])
        rospy.loginfo("starting: '%s' ..." % cmd)
        app = ManagedApplication(cmd, window=mplayer_window)
        app.set_state(ApplicationState.VISIBLE)
        rospy.loginfo("Application '%s' started." % cmd)
        return app

    def _shutdown(self):
        rospy.loginfo("Stopping managed applications ...")
        for app_id, app in self.apps.items():
            rospy.loginfo("Stopping app id '%s' ..." % app_id)
            app.proc.stop()
            del self.apps[app_id]

    def _get_instance(self):
        """
        Returns prefix plus the name of the ROS node.

        """
        return '_mplayer_instance_' + rospy.get_name().strip('/')


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
