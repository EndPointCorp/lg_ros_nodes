#!/usr/bin/env python

import os
import json
import time
import rospy

from std_msgs.msg import String
from lg_common.msg import WindowGeometry
from appctl_support import ProcController
from lg_common.msg import ApplicationState
from lg_media.srv import MediaAppsInfoResponse
from lg_common.helpers import get_app_instances_ids
from lg_common import ManagedApplication, ManagedWindow
from lg_common.helpers import get_app_instances_to_manage


ROS_NODE_NAME = "lg_media"
DEFAULT_APP = "mplayer"
DEFAULT_ARGS = "-idle -slave -cache 256 -quiet -osdlevel 0 -nomouseinput -nograbpointer"
SRV_QUERY = '/'.join(('', ROS_NODE_NAME, "query"))


class ManagedMplayer(ManagedApplication):
    """
    Instance corresponds to a mplayer application managed entity.

    """
    def __init__(self, fifo_path, url, slug, window, respawn=True):
        self.window = window
        self.fifo_path = fifo_path
        self.url = url
        self.slug = slug
        self.respawn = respawn

        super(ManagedMplayer, self).__init__(window=window,
                                             respawn=self.respawn,
                                             cmd=self._build_cmd())

    def __str__(self):
        """
        String representation
        """
        r = "state='%s' FIFO='%s' URL='%s'" % (self.state, self.fifo_path, self.url)
        return r

    def __repr__(self):
        """
        Direct call representation
        """
        r = "state='%s' FIFO='%s' URL='%s'" % (self.state, self.fifo_path, self.url)
        return r

    def _build_cmd(self):
        """
        Mplayer needs to have geometry supplied thru command arguments - window manager rules won't affect it
        It also needs manual fifo specification.
        """
        cmd = []
        cmd.extend([rospy.get_param("~application_path", DEFAULT_APP)])
        cmd.extend(rospy.get_param("~application_flags", DEFAULT_ARGS).split())

        cmd.extend(['-geometry', '{0}x{1}+{2}+{3}'.format(self.window.geometry.width,
                                                          self.window.geometry.height,
                                                          self.window.geometry.x,
                                                          self.window.geometry.y)])
        cmd.extend(["-input", "file=%s" % self.fifo_path])
        cmd.extend([self.url])
        rospy.logdebug("Mplayer POOL: mplayer cmd: %s" % cmd)
        return cmd

    def close(self):
        """
        Shut down mplayer and make sure it didn't lave anything behind
        """
        self.set_state(ApplicationState.STOPPED)
        os.unlink(self.fifo_path)
        return True

    def execute_command(self, command):
        """
        Execute any command that can set mplayer's internal variable on the fly:
            http://www.mplayerhq.hu/DOCS/tech/slave.txt and `mplayer -input cmdlist`
        Remember that not all parameters are settable
        """
        with open(self.fifo_path, 'w') as fifo:
            fifo.write(command + "\n")
            return True

    def change_url(self, url):
        """
        Load another movie using the same instance of mplayer
        """
        with open(self.fifo_path, 'w') as fifo:
            rospy.logdebug("Changing Mplayer %s url to %s" % (self.slug, url))
            fifo.write('loadfile %s\n' % url)

    def update_geometry(self, geometry):
        """
        Unfortunately updating geometry cannot be done from mplayer's instance
        """
        pass


class MplayerPool(object):
    """
    Manages pool of MplayerInstances in self.mplayers
    dict(id => MplayerInstance)
    """

    def __init__(self, viewport_name):
        self.mplayers = {}  # key: app id, value: MplayerInstance
        self.viewport_name = viewport_name

    def _unpack_incoming_mplayers(self, mplayers):
        """
        Converts incoming AdhocMedias to a dictionary where keys are ids
        It will filter out all 'non-mplayer' adhoc medias
        """
        return {m.id: m for m in mplayers if m.media_type == 'video'}

    def handle_ros_message(self, data):
        """
        Handles AdhocMedias messages and manages MplayerInstances in MplayerPool
        """

        incoming_mplayers = self._unpack_incoming_mplayers(data.medias)
        incoming_mplayers_ids = set(incoming_mplayers.keys())

        current_mplayers_ids = get_app_instances_ids(self.mplayers)

        # mplayers to remove
        for mplayer_pool_id in get_app_instances_to_manage(current_mplayers_ids,
                                                           incoming_mplayers_ids,
                                                           manage_action='remove'):
            rospy.logdebug("Removing mplayer id %s" % mplayer_pool_id)
            self._remove_mplayer(mplayer_pool_id)

        # mplayers to create
        for mplayer_pool_id in get_app_instances_to_manage(current_mplayers_ids,
                                                           incoming_mplayers_ids,
                                                           manage_action='create'):
            rospy.logdebug("Creating mplayer with id %s" % mplayer_pool_id)
            self._create_mplayer(mplayer_pool_id, incoming_mplayers[mplayer_pool_id])

        # mplayers to update
        for mplayer_pool_id in get_app_instances_to_manage(current_mplayers_ids,
                                                           incoming_mplayers_ids,
                                                           manage_action='update'):
            rospy.logdebug("Updating mplayer with id %s" % mplayer_pool_id)
            self._update_mplayer(mplayer_pool_id, incoming_mplayers[mplayer_pool_id])

        return True

    def get_media_apps_info(self, request):
        """
        Connected to a service call, returns content of the internal
        container tracking currently running managed applications.

        """
        d = {app_id: str(app_info) for app_id, app_info in self.mplayers.items()}
        return MediaAppsInfoResponse(json=json.dumps(d))

    def _create_mplayer(self, mplayer_id, incoming_mplayer):
        """
        Start a ManagedApplication instance according to the details in the
        media argument and return process instance, FIFO file (full path) to
        drive the mplayer application and resource URL.

        """
        fifo_path = self._create_fifo(mplayer_id)
        geometry = WindowGeometry(x=incoming_mplayer.geometry.x,
                                  y=incoming_mplayer.geometry.y,
                                  width=incoming_mplayer.geometry.width,
                                  height=incoming_mplayer.geometry.height)

        mplayer_window = ManagedWindow(geometry=geometry,
                                       w_instance="Mplayer \\({}\\)".format(mplayer_id),
                                       w_class="Mplayer \\({}\\)".format(mplayer_id))

        if incoming_mplayer.on_finish == "nothing" or incoming_mplayer.on_finish == "close":
            respawn = False
        else:
            respawn = True
        mplayer = ManagedMplayer(fifo_path=fifo_path,
                                 url=incoming_mplayer.url,
                                 slug=mplayer_id,
                                 window=mplayer_window,
                                 respawn=respawn)

        mplayer.set_state(ApplicationState.VISIBLE)

        rospy.logdebug("MPlayer Pool: started new mplayer instance %s on viewport %s with id %s" % (self.viewport_name, incoming_mplayer, mplayer_id))
        self.mplayers[mplayer_id] = mplayer

        return True

    def _create_fifo(self, mplayer_id):
        name = "lg_%s_%s.fifo" % (mplayer_id, time.time())
        path = os.path.join("/tmp", name)
        os.mkfifo(path)
        rospy.logdebug("Created FIFO file '%s'" % path)
        return path

    def _update_mplayer(self, mplayer_pool_id, updated_mplayer):
        """
        Mplayer will re-use the existing instance if geometry of new mplayer is identical but only the url is different.
        In this case it will just load the url via fifo.

        If geometry is different then new mplayer will be started and the old one will get killed.
        """

        current_mplayer = self.mplayers[mplayer_pool_id]
        rospy.logdebug("MPlayer Pool %s: I'm going to update mplayer_pool_id: %s and instance:" % (mplayer_pool_id, current_mplayer))
        future_url = updated_mplayer.url
        current_mplayer.change_url(future_url)

        future_geometry = WindowGeometry(x=updated_mplayer.geometry.x,
                                         y=updated_mplayer.geometry.y,
                                         width=updated_mplayer.geometry.width,
                                         height=updated_mplayer.geometry.height)

        current_geometry = current_mplayer.window.geometry

        if (current_geometry.x != future_geometry.x) or \
                (current_geometry.y != future_geometry.y) or \
                (current_geometry.width != future_geometry.width) or\
                (current_geometry.height != future_geometry.height):

            rospy.logdebug("MPlayer Pool: New geometry (%s) is different from the old geometry (%s) - killing and creating new instance" % (future_geometry, current_geometry))
            self._remove_mplayer(mplayer_pool_id)
            self._create_mplayer(mplayer_pool_id, updated_mplayer)

    def _remove_mplayer(self, mplayer_pool_id):
        """
        Wipe out mplayer instance - both from the screen and memory
        """
        mplayer_instance = self.mplayers[mplayer_pool_id]
        rospy.logdebug("Stopping app id '%s', Mplayer instance %s:" % (mplayer_pool_id, mplayer_instance))
        mplayer_instance.close()
        del self.mplayers[mplayer_pool_id]

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
