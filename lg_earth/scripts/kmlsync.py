#!/usr/bin/env python

import rospy

from flask import Flask
from lg_earth import KMLSyncServer
from lg_common.webapp import ros_flask_spin

def main():
    rospy.init_node('kmlsync_server', anonymous=True)
    host = rospy.get_param('~kmlsync_listen_host', '127.0.0.1')
    port = rospy.get_param('~kmlsync_listen_port', 8765)
    app = Flask(__name__)
    ros_flask_spin(app, host=host, port=port, debug=True, use_reloader=False, flask_classes=[KMLSyncServer])


if __name__ == '__main__':
    main()
