#!/usr/bin/env python

import rospy

from flask import Flask
from tendo import singleton
from lg_earth import KMLSyncServer
from multiprocessing import Process
from lg_common.webapp import ros_flask_spin
from lg_common.helpers import write_log_to_file

def main():
    write_log_to_file("---------START-------------")
    rospy.init_node('kmlsync_server', anonymous=True)

    host = rospy.get_param('~kmlsync_listen_host', '127.0.0.1')
    port = rospy.get_param('~kmlsync_listen_port', 8765)
    app = Flask(__name__)
    ros_flask_spin(app, host=host, port=port, debug=True, flask_classes=[KMLSyncServer])


if __name__ == '__main__':
    main()
