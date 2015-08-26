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
from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_app_instances_ids
from lg_common import ManagedApplication, ManagedWindow
from lg_common.helpers import get_app_instances_to_manage


DEFAULT_APP = "mplayer"

class ManagedMplayer(ManagedApplication):
    def __init__(self, fifo_path, url, slug, window):
        self.window = window
        self.fifo_path = fifo_path
        self.url = url
        self.slug = slug

        super(ManagedMplayer, self).__init__(
                window=window,
                cmd=self._build_cmd()
        )

    def __str__(self):
        r = "state='%s' FIFO='%s' URL='%s'" % (self.app.state, self.fifo_path, self.url)
        return r

    def __str__(self):
        r = "state='%s' FIFO='%s' URL='%s'" % (self.app.state, self.fifo_path, self.url)
        return r

    def _build_cmd(self):
        cmd = []
        cmd.extend([rospy.get_param("~application_path", DEFAULT_APP)])
        cmd.extend(rospy.get_param("~application_flags").split())

        cmd.extend(['-geometry', '{0}x{1}+{2}+{3}'.format(self.window.geometry.width,
                                                           self.window.geometry.height,
                                                           self.window.geometry.x,
                                                           self.window.geometry.y)])
        cmd.extend(["-input", "file=%s" % self.fifo_path])
        cmd.extend([self.url])
        rospy.loginfo("Mplayer POOL: mplayer cmd: %s" % cmd)
        return cmd

    def close(self):
        self.set_state(ApplicationState.STOPPED)
        os.unlink(self.fifo_path)
        return True

    def execute_command(self, command):
        """
        http://www.mplayerhq.hu/DOCS/tech/slave.txt and `mplayer -input cmdlist`
        """
        with open(self.fifo_path, 'w') as fifo:
            fifo.write(command)
            return True

    def change_media(self, url):
        #write to fifo
        # loadfile <url>
        pass

    def update_width_height(self, width, height):
        #write to fifo:
        # width <width>
        # height <height>
        #
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

        incoming_mplayers      = self._unpack_incoming_mplayers(data.medias)
        incoming_mplayers_ids  = set(incoming_mplayers.keys())

        current_mplayers_ids   = get_app_instances_ids(self.mplayers)

        # mplayers to remove
        for mplayer_pool_id in get_app_instances_to_manage(current_mplayers_ids,
                                                           incoming_mplayers_ids,
                                                           manage_action='remove'):
            rospy.loginfo("Removing mplayer id %s" % mplayer_pool_id)
            self._remove_mplayer_instance(mplayer_pool_id)

        # mplayers to create
        for mplayer_pool_id in get_app_instances_to_manage(current_mplayers_ids,
                                                           incoming_mplayers_ids,
                                                           manage_action='create'):
            rospy.loginfo("Creating mplayer with id %s" % mplayer_pool_id)
            self._create_mplayer(mplayer_pool_id, incoming_mplayers[mplayer_pool_id])

        # mplayers to update
        for mplayer_pool_id in get_app_instances_to_manage(current_mplayers_ids,
                                                           incoming_mplayers_ids,
                                                           manage_action='update'):
            rospy.loginfo("Updating mplayer with id %s" % mplayer_pool_id)
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
        geometry = WindowGeometry(x=incoming_mplayer.geometry.x,
                                  y=incoming_mplayer.geometry.y,
                                  width=incoming_mplayer.geometry.width,
                                  height=incoming_mplayer.geometry.height)

        mplayer_window = ManagedWindow(geometry=geometry,
                                       w_instance="mplayer {}".format(mplayer_id))


        fifo_path = self._create_fifo(mplayer_id)

        mplayer = ManagedMplayer(fifo_path=fifo_path,
                                 url=incoming_mplayer.url,
                                 slug=mplayer_id,
                                 window=mplayer_window)

        mplayer.set_state(ApplicationState.VISIBLE)
        rospy.loginfo("Mplayer POOL: started new mplayer instance %s on viewport %s with id %s" % (self.viewport_name, incoming_mplayer, mplayer_id))
        self.mplayers[mplayer_id] = incoming_mplayer

        return True

    def _create_fifo(self, mplayer_id):
        name = "lg_%s_%s.fifo" % (mplayer_id, time.time())
        path = os.path.join("/tmp", name)
        os.mkfifo(path)
        rospy.loginfo("Created FIFO file '%s'" % path)
        return path

    def _update_mplayer(self, mplayer_pool_id):
        """
        Well well well :)
        TODO (wzin): make the real update happen rather then destroy + create
        """
        self._remove_mplayer_instance(self, mplayer_pool_id)
        self._create_mplayer_instance(self, mplayer_pool_id)
        return True

    def _remove_mplayer(self, mplayer_pool_id):
        mplayer_instance = self.mplayers[mplayer_pool_id]
        rospy.loginfo("Stopping app id '%s', Mplayer instance %s:" % (mplayer_pool_id, mplayer_instance))
        mplayer_instance.close()
        del self.mplayers[app_id]

    #####################

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
