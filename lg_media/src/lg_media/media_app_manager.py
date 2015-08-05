#!/usr/bin/env python
"""
Adhoc media service.

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

# call service
rosservice call /lg_media/query

"""

import os
import json
import time

import rospy
from std_msgs.msg import String

from appctl_support import ProcController
from lg_common import ManagedApplication, ManagedWindow
from lg_common.msg import ApplicationState
from lg_common.msg import WindowGeometry
from lg_media.srv import MediaAppsInfoResponse


ROS_NODE_NAME = "lg_media"
DEFAULT_VIEWPORT = "test_viewport_0"
DEFAULT_APP = "mplayer"
SRV_QUERY = '/'.join(('', ROS_NODE_NAME, "query"))


class AppInstance(object):
    def __init__(self, app, fifo_path):
        self.app = app
        self.fifo_path = fifo_path

    def __str__(self):
        r = "cmd:%s state:%s FIFO:%s" % (self.app.cmd, self.app.state, self.fifo_path)
        return r


class MediaService(object):
    """

    """

    @property
    def app_cmd(self):
        cmd = [rospy.get_param("~application_path", DEFAULT_APP)]
        cmd.extend(rospy.get_param("~application_flags").split())
        return cmd

    def __init__(self):
        self.apps = {}  # key: app id, value: AppInstance

    def listener(self, data):
        """
        Listener on a ROS topic.

        """
        rospy.logdebug("'listener' invoked, received data: '%s'" % data)

        if len(data.medias) == 0:
            rospy.logdebug("Media array empty, shutting existing applications down ...")
            self._shutdown()
            return

        received_ids = [media.id for media in data.medias]

        # update applications (intersection of received_ids and existing app ids)
        for media in data.medias:
            if media.id in self.apps.keys():
                fifo = self.apps[media.id].fifo_path
                rospy.logdebug("App id '%s' exists, updating, FIFO: '%s') ..." % (media.id, fifo))
                os.system("echo 'loadfile %s' > %s" % (media.url, fifo))
                # TODO
                # this way it doesn't work ...
                #fifo_desc = open(fifo, 'w')
                #fifo_desc.write('loadfile %s' % media.url)
                #fifo_desc.close()
                # TODO
                # update geometry via change_rectangle <val1> <val2> commands
                rospy.logdebug("Updated instance '%s' with URL '%s'" % (media.id, media.url))

        # shutdown applications
        to_shutdown = set(self.apps.keys()) - set(received_ids)
        [self._shutdown_instance(app_id) for app_id in to_shutdown]

        # create applications
        new_apps = {}
        for media in data.medias:
            if media.id in set(received_ids) - set(self.apps.keys()):
                rospy.logdebug("App id '%s' doesn't exist, starting ..." % media.id)
                new_apps[media.id] = AppInstance(*self._start_and_get_app(media))
        self.apps.update(new_apps)

    def get_media_apps_info(self, request):
        """
        Connected to a service call, returns content of the internal
        container tracking currently running managed applications.

        """
        d = {app_id: str(app_info) for app_id, app_info in self.apps.items()}
        return MediaAppsInfoResponse(json=json.dumps(d))

    def _start_and_get_app(self, media):
        """
        Start a ManagedApplication instance according to the details in the
        media argument and return instance and fifo file descriptor to drive
        the application.

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
        name = "lg-%s-%s.fifo" % (self.__class__.__name__, time.time())
        path = os.path.join("/tmp", name)
        os.mkfifo(path)
        rospy.loginfo("Created FIFO file '%s'" % path)
        cmd.extend(["-input", "file=%s" % path])
        cmd.extend([media.url])
        rospy.loginfo("starting: '%s' ..." % cmd)
        app = ManagedApplication(cmd, window=mplayer_window)
        app.set_state(ApplicationState.VISIBLE)
        rospy.loginfo("Application '%s' started." % cmd)
        return app, path

    def _shutdown_instance(self, app_id):
        app_instance = self.apps[app_id]
        rospy.loginfo("Stopping app id '%s', FIFO: '%s' ..." % (app_id, app_instance.fifo_path))
        app_instance.app.proc.stop()
        os.unlink(app_instance.fifo_path)
        assert not os.path.exists(app_instance.fifo_path)
        del self.apps[app_id]

    def _shutdown(self):
        rospy.loginfo("Stopping all (num: %s) managed applications ..." % (len(self.apps)))
        for app_id in self.apps.keys()[:]:
            self._shutdown_instance(app_id)

    def _get_instance(self):
        """
        Returns prefix plus the name of the ROS node.

        """
        return '_mplayer_instance_' + rospy.get_name().strip('/')


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
