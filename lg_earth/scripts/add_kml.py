#!/usr/bin/env python

from std_msgs.msg import String
from lg_common.srv import USCSMessage
from interactivespaces_msgs.msg import GenericMessage

import SimpleHTTPServer
import SocketServer
import threading
import tempfile
import rospy
import json
import copy
import os

DEFAULT_VIEWPORTS = ['left_three', 'left_two', 'left_one', 'center',
                     'right_one', 'right_two', 'right_three']
DEFAULT_EARTH_INSTANCE = {u'activity': u'earth',
   u'activity_config': {},
   u'assets': [],
   u'height': 1920,
   u'presentation_viewport': u'CHANGE_ME',
   u'slug': -1875729098,
   u'width': 1080,
   u'x_coord': 0,
   u'y_coord': 0}


class KMLAdder():
    def __init__(self, uscs_service, director_pub, port, hostname='localhost', viewports=None):
        self.serve_dir = tempfile.mktemp()
        self.uscs_service = uscs_service
        self.director_pub = director_pub
        self.viewports = viewports
        if self.viewports is None:
            self.viewports = DEFAULT_VIEWPORTS
        self.port = port
        self.server = threading.Thread(target=self._serve)

        os.mkdir(self.serve_dir)
        self.server.start()

    def handle_kml(self, msg):
        kml = msg.data

        filename = tempfile.mktemp(dir=self.serve_dir)
        with open(filename, 'w') as f:
            f.write(kml)

        current_scene = self.uscs_service.call().message
        current_scene = json.loads(current_scene)
        self.add_earths(current_scene)
        for window in current_scene['windows']:
            if window['activity'] != 'earth':
                continue
            window['assets'].append('http://{}:{}/{}'.format(self.hostname, self.port, os.path.basename(filename)))
            print 'adding ("http://localhost:{}/{}" to viewport {}'.format(self.port, os.path.basename(filename), window['presentation_viewport'])
        new_msg = GenericMessage()
        new_msg.type = 'json'
        new_msg.message = json.dumps(current_scene)
        self.director_pub.publish(new_msg)

    def _serve(self):
        os.chdir(self.serve_dir)
        Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

        self.httpd = SocketServer.TCPServer(("", self.port), Handler)

        print "serving at port", self.port
        self.httpd.serve_forever()

    def add_earths(self, scene):
        for viewport in self.viewports:
            flag = False
            for window in scene['windows']:
                if window['activity'] == 'earth' and window['presentation_viewport'] == viewport:
                    flag = True
            # if no instance of earth w/ our current viewport is found
            # we add one and give it our viewport
            if flag is False:
                print('appending to viewport {}'.format(viewport))
                scene['windows'].append(copy.deepcopy(DEFAULT_EARTH_INSTANCE))
                scene['windows'][-1]['presentation_viewport'] = viewport

    def shutdown(self):
        self.httpd.shutdown()
        self.server.join()

def main():
    rospy.init_node('add_kml')

    director_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
    uscs_service = rospy.ServiceProxy('/uscs/message', USCSMessage)

    hostname = rospy.get_param('~hostname', 'localhost')
    port = rospy.get_param('~port', 18111)

    k = KMLAdder(uscs_service, director_pub, port, hostname)

    rospy.Subscriber('/lg_earth/add_kml', String, k.handle_kml)

    rospy.on_shutdown(k.shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()
