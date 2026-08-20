#!/usr/bin/env python3

from collections import deque
from functools import partial
import http.server
import json
import os
import socketserver
import tempfile
import threading

import rospy
from geometry_msgs.msg import Pose
from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.srv import USCSMessage
from std_msgs.msg import String

from lg_common.helpers import run_with_influx_exception_handler
from lg_earth.tour import (attach_leader_tour, build_tour_kml,
                           pose_camera_kml, pose_lookat_kml)


NODE_NAME = 'earth_tour'


class ReusableTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


class LeaderTourPlayer(object):
    def __init__(self, uscs_service, director_pub, hostname, port,
                 viewport='center'):
        self.uscs_service = uscs_service
        self.director_pub = director_pub
        self.hostname = hostname
        self.port = port
        self.viewport = viewport
        self.sequence = 0
        self.pending_camera = None
        self.pending_lock = threading.Lock()
        self.play_lock = threading.Lock()
        self.files = deque()
        self.serve_dir = tempfile.mkdtemp(prefix='earth-tour-')

        handler = partial(http.server.SimpleHTTPRequestHandler,
                          directory=self.serve_dir)
        self.httpd = ReusableTCPServer(('', self.port), handler)
        self.server = threading.Thread(target=self.httpd.serve_forever)
        self.server.daemon = True
        self.server.start()

    @property
    def url_prefix(self):
        return 'http://{}:{}/'.format(self.hostname, self.port)

    def handle_camera(self, message):
        self.queue_camera(pose_camera_kml(message))

    def handle_lookat(self, message):
        self.play(pose_lookat_kml(message))

    def handle_fragment(self, message):
        self.play(message.data)

    def handle_sync_camera(self, message):
        self.queue_camera(pose_camera_kml(message))

    def queue_camera(self, fragment):
        # Touch controllers and background renderers can update faster than
        # Earth loads tours. One shared slot means intermediate poses disappear
        # instead of becoming an interrupted-fly-to playback queue.
        with self.pending_lock:
            self.pending_camera = fragment

    def flush_camera(self, _event=None):
        # Do not let timer callbacks line up behind a slow Director/KML load.
        # Leaving the slot untouched makes the next tick use the newest pose.
        if not self.play_lock.acquire(False):
            return
        try:
            with self.pending_lock:
                fragment = self.pending_camera
                self.pending_camera = None
            if fragment is not None:
                self._play(fragment)
        finally:
            self.play_lock.release()

    def play(self, fragment):
        # A direct touchscreen request and the background timer can arrive on
        # different callback threads; serialize the scene/file replacement.
        with self.play_lock:
            self._play(fragment)

    def _play(self, fragment):
        self.sequence += 1
        tour_name = 'earth-flyto-{}'.format(self.sequence)
        filename = '{}.kml'.format(tour_name)
        path = os.path.join(self.serve_dir, filename)
        kml = build_tour_kml(fragment, tour_name=tour_name, duration=0)
        with open(path, 'w') as destination:
            destination.write(kml)

        scene = json.loads(self.uscs_service.call().message)
        scene = attach_leader_tour(
            scene,
            self.url_prefix + filename,
            self.url_prefix,
            viewport=self.viewport,
        )
        message = GenericMessage()
        message.type = 'json'
        message.message = json.dumps(scene)
        self.director_pub.publish(message)

        self.files.append(path)
        while len(self.files) > 3:
            old_path = self.files.popleft()
            try:
                os.remove(old_path)
            except OSError:
                pass

    def shutdown(self):
        self.httpd.shutdown()
        self.server.join()


def main():
    rospy.init_node(NODE_NAME)
    director_pub = rospy.Publisher('/director/scene', GenericMessage,
                                   queue_size=1)
    uscs_service = rospy.ServiceProxy('/uscs/message', USCSMessage,
                                      persistent=False)
    player = LeaderTourPlayer(
        uscs_service=uscs_service,
        director_pub=director_pub,
        hostname=rospy.get_param('~hostname', 'localhost'),
        port=rospy.get_param('~port', 18112),
        viewport=rospy.get_param('~viewport', 'center'),
    )

    rospy.Subscriber('/earth/query/flyto_pose_camera', Pose,
                     player.handle_camera, queue_size=1)
    rospy.Subscriber('/earth/query/flyto_pose_lookat', Pose,
                     player.handle_lookat, queue_size=1)
    rospy.Subscriber('/earth/query/flyto_kml', String,
                     player.handle_fragment, queue_size=1)
    rospy.Subscriber('/earth/query/sync_pose_camera', Pose,
                     player.handle_sync_camera, queue_size=1)

    sync_interval = rospy.get_param('~sync_interval', 0.1)
    rospy.Timer(rospy.Duration.from_sec(sync_interval),
                player.flush_camera)
    rospy.on_shutdown(player.shutdown)
    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
