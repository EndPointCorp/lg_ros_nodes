#!/usr/bin/env python

from std_msgs.msg import String, Empty
from lg_common.srv import USCSMessage
from lg_common.msg import StringArray
from interactivespaces_msgs.msg import GenericMessage

import SimpleHTTPServer
import SocketServer
import threading
import tempfile
import rospy
import json
import copy
import os
import re
import binascii

DEFAULT_VIEWPORTS = ['left_three', 'left_two', 'left_one', 'center',
                     'right_one', 'right_two', 'right_three']
DEFAULT_EARTH_INSTANCE = {
    u'activity': u'earth',
    u'activity_config': {},
    u'assets': [],
    u'height': 1920,
    u'presentation_viewport': u'CHANGE_ME',
    u'slug': -1875729098,
    u'width': 1080,
    u'x_coord': 0,
    u'y_coord': 0
}

kml_id_pattern = re.compile('<kml .* id=\"()\".*>')


def get_kml_id(kml):
    """
    if <kml> tag has id attribute returns it value
    othervise return unsigned crc32 of kml string
    """
    id_match = kml_id_pattern.search(kml, re.IGNORECASE)
    if id_match:
        return id_match.group(1)

    return hex(binascii.crc32(kml) % (1 << 32))


def get_match_any_starts_with(prefixes):
    def matcher(test_string):
        for prefix in prefixes:
            if test_string.startswith(prefix):
                return True
        return False
    
    return matcher


class KMLAdder():
    def __init__(self, uscs_service, director_pub, added_kml_pub, port, hostname='localhost', viewports=None):
        self.serve_dir = tempfile.mktemp()
        self.uscs_service = uscs_service
        self.director_pub = director_pub
        self.added_kml_pub = added_kml_pub
        self.id_to_file = dict()
        self.hostname = hostname
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
        
        kml_id = get_kml_id(kml)
        if kml_id not in self.id_to_file:
            self.id_to_file[kml_id] = list()
        
        # Keep track of files for easier remove by id
        self.id_to_file[kml_id].append(os.path.basename(filename))

        current_scene = self.uscs_service.call().message
        current_scene = json.loads(current_scene)
        self.add_earths(current_scene)
        for window in current_scene['windows']:
            if window['activity'] != 'earth':
                continue
            window['assets'].append(self.formatURL(filename))
        new_msg = GenericMessage()
        new_msg.type = 'json'
        new_msg.message = json.dumps(current_scene)
        self.director_pub.publish(new_msg)
        self.added_kml_pub.publish(StringArray(list(self.id_to_file.keys())))

    def formatURL(self, filename):
        return 'http://{}:{}/{}'.format(self.hostname, self.port, os.path.basename(filename))

    def clear_kmls(self, msg):
        current_scene = self.uscs_service.call().message
        current_scene = json.loads(current_scene)

        ids = msg.strings if msg.strings else None

        matcher = None
        if ids:
            files = [name for name in names for names in self.id_to_file]
            for id in ids:
                self.id_to_file.pop(id)
            urls_to_remove = [self.formatURL(filename) for filename in files]
            matcher = get_match_any_starts_with(urls_to_remove)
        else:
            # Remove all additional kmls
            self.id_to_file = dict()
            matcher = get_match_any_starts_with([self.formatURL(self.serve_dir)])
        
        for window in current_scene['windows']:
            if window['activity'] == 'earth':
                window['assets'] = [a for a in window['assets'] if not matcher(a)]
        
        new_msg = GenericMessage()
        new_msg.type = 'json'
        new_msg.message = json.dumps(current_scene)
        self.director_pub.publish(new_msg)
        self.added_kml_pub.publish(StringArray(list(self.id_to_file.keys())))

    def _serve(self):
        os.chdir(self.serve_dir)
        Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

        self.httpd = SocketServer.TCPServer(("", self.port), Handler)

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
                scene['windows'].append(copy.deepcopy(DEFAULT_EARTH_INSTANCE))
                scene['windows'][-1]['presentation_viewport'] = viewport

    def shutdown(self):
        self.httpd.shutdown()
        self.server.join()


def main():
    rospy.init_node('add_kml')

    director_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
    added_kml_pub = rospy.Publisher('/lg_earth/added_kml', StringArray, latch=True, queue_size=1)
    
    uscs_service = rospy.ServiceProxy('/uscs/message', USCSMessage)

    hostname = rospy.get_param('~hostname', 'localhost')
    port = rospy.get_param('~port', 18111)

    k = KMLAdder(uscs_service, director_pub, added_kml_pub, port, hostname)

    rospy.Subscriber('/lg_earth/add_kml', String, k.handle_kml)
    rospy.Subscriber('/lg_earth/clear_kml', StringArray, k.clear_kmls)

    rospy.on_shutdown(k.shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()
