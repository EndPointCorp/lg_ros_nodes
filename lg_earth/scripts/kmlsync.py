#!/usr/bin/env python

import rospy

from flask import Flask
from lg_earth import KMLSyncServer
from lg_common.webapp import ros_flask_spin

app = Flask(__name__)

def main():
    rospy.init_node('kmlsync_server', anonymous=True)

    host = rospy.get_param('~kmlsync_listen_host', '127.0.0.1')
    port = rospy.get_param('~kmlsync_listen_port', 8765)

    ros_flask_spin(app, host=host, port=port, debug=True, flask_classes=[KMLSyncServer])

    rospy.spin()

if __name__ == '__main__':
    main()
