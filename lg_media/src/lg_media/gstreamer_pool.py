import os
import json
import time
import threading
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
DEFAULT_APP = "gst_video_sync"
DEFAULT_ARGS = " -a 10.42.42.255"
SRV_QUERY = '/'.join(('', ROS_NODE_NAME, "query"))


class ManagedGstreamer(ManagedApplication):
    """
    Instance corresponds to a gstreamer application managed entity.

    """
    def __init__(self, url, slug, window, respawn=True, extra_args=''):
        self.window = window
        self.url = url
        self.slug = slug
        self.respawn = respawn
        self.extra_args = extra_args

        super(ManagedGstreamer, self).__init__(window=window,
                                               respawn=self.respawn,
                                               cmd=self._build_cmd())

    def __str__(self):
        """
        String representation
        """
        r = "state='%s' URL='%s'" % (self.state, self.url)
        return r

    def __repr__(self):
        """
        Direct call representation
        """
        r = "state='%s' URL='%s'" % (self.state, self.url)
        return r

    def _build_cmd(self):
        cmd = []
        cmd.extend([rospy.get_param("~application_path", DEFAULT_APP)])
        cmd.extend(rospy.get_param("~application_flags", DEFAULT_ARGS).split())
        if self.extra_args != '':
            cmd.extend(self.extra_args.split())

        cmd.extend(["-w", str(self.slug)])
        cmd.extend(["-u", self.url])
        #if self.respawn:
        #    cmd.extend(["-loop", "0"])
        rospy.logdebug("GStreamer POOL: gst_video_sync cmd: %s" % cmd)
        return cmd

    def execute_command(self, command):
        raise NotImplementedError()

    def change_url(self, url):
        raise NotImplementedError()

    def update_geometry(self, geometry):
        raise NotImplementedError()


class GstreamerPool(object):
    """
    Manages pool of GstreamerInstances in self.gstreamers
    dict(id => GstreamerInstance)
    """

    def __init__(self, viewport_name):
        self.gstreamers = {}  # key: app id, value: GstreamerInstance
        self.viewport_name = viewport_name
        self.lock = threading.Lock()
        rospy.on_shutdown(self.clear)

    def clear(self):
        with self.lock:
            for k in self.gstreamers.keys():
                self.gstreamers[k].close()
                del self.gstreamers[k]

    def _unpack_incoming_gstreamers(self, gstreamers):
        """
        Converts incoming AdhocMedias to a dictionary where keys are ids
        It will filter out all 'non-gstreamer' adhoc medias
        """
        return {m.id: m for m in gstreamers if m.media_type == 'video'}

    def _partition_existing_medias(self, incoming_medias):
        """
        Determine which media id's belong to existing assets.
        """
        existing_media_urls = [m.url for m in self.gstreamers.values()]

        def media_exists(media):
            return media.url in existing_media_urls

        existing_media_ids = [m.id for m in incoming_medias if media_exists(m)]
        fresh_media_ids = [m.id for m in incoming_medias if not media_exists(m)]

        return existing_media_ids, fresh_media_ids

    def handle_ros_message(self, data):
        """
        Handles AdhocMedias messages and manages GstreamerInstances in GstreamerPool
        """
        with self.lock:
            incoming_gstreamers = self._unpack_incoming_gstreamers(data.medias)

            incoming_gstreamers_ids = set(incoming_gstreamers.keys())

            current_gstreamers_ids = get_app_instances_ids(self.gstreamers)

            existing_media_ids, fresh_media_ids = self._partition_existing_medias(incoming_gstreamers.values())

            # gstreamers to remove
            for gstreamer_pool_id in current_gstreamers_ids:
                if gstreamer_pool_id in existing_media_ids:
                    rospy.loginfo("Media already playing: %s" % gstreamer_pool_id)
                    continue
                rospy.loginfo("Removing gstreamer id %s" % gstreamer_pool_id)
                self._remove_gstreamer(gstreamer_pool_id)

            # gstreamers to create
            for gstreamer_pool_id in fresh_media_ids:
                rospy.loginfo("Creating gstreamer with id %s" % gstreamer_pool_id)
                self._create_gstreamer(gstreamer_pool_id, incoming_gstreamers[gstreamer_pool_id])

            return True

    def get_media_apps_info(self, request):
        """
        Connected to a service call, returns content of the internal
        container tracking currently running managed applications.

        """
        with self.lock:
            d = {app_id: str(app_info) for app_id, app_info in self.gstreamers.items()}
            return MediaAppsInfoResponse(json=json.dumps(d))

    def _create_gstreamer(self, gstreamer_id, incoming_gstreamer):
        """
        Start a ManagedApplication instance according to the details in the
        media argument and return process instance, FIFO file (full path) to
        drive the gstreamer application and resource URL.

        """
        geometry = WindowGeometry(x=incoming_gstreamer.geometry.x,
                                  y=incoming_gstreamer.geometry.y,
                                  width=incoming_gstreamer.geometry.width,
                                  height=incoming_gstreamer.geometry.height)

        gstreamer_window = ManagedWindow(geometry=geometry,
                                         w_name=str(gstreamer_id))

        if incoming_gstreamer.on_finish == "nothing" or incoming_gstreamer.on_finish == "close":
            respawn = False
        else:
            respawn = True
        gstreamer = ManagedGstreamer(url=incoming_gstreamer.url,
                                     slug=gstreamer_id,
                                     window=gstreamer_window,
                                     respawn=respawn,
                                     extra_args=incoming_gstreamer.extra_args)

        gstreamer.set_state(ApplicationState.VISIBLE)

        rospy.logdebug("MPlayer Pool: started new gstreamer instance %s on viewport %s with id %s" % (self.viewport_name, incoming_gstreamer, gstreamer_id))

        self.gstreamers[gstreamer_id] = gstreamer

        return True

    def _remove_gstreamer(self, gstreamer_pool_id):
        """
        Wipe out gstreamer instance - both from the screen and memory
        """
        gstreamer_instance = self.gstreamers[gstreamer_pool_id]
        rospy.logdebug("Stopping app id '%s', Gstreamer instance %s:" % (gstreamer_pool_id, gstreamer_instance))
        gstreamer_instance.close()
        del self.gstreamers[gstreamer_pool_id]

    def handle_soft_relaunch(self, *args, **kwargs):
        gstreamers = self.gstreamers.keys()
        for gstreamer in gstreamers:
            self._remove_gstreamer(gstreamer)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
