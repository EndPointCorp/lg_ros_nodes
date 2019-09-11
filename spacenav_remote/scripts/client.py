#! /usr/bin/env python3

import json
import socket
import argparse
import time
from math import fabs

from spnav import spnav_open, spnav_poll_event, spnav_close
from spnav import SPNAV_EVENT_MOTION

parser = argparse.ArgumentParser()
parser.add_argument("-q", "--quite", action="store_true", help="Do not print messages")
parser.add_argument("-p", "--port", default=6564, help="port, default 6564", type=int)
parser.add_argument("-r", "--reconnect", action="store_true", help="Try to reconnect")
parser.add_argument("-s", "--host", default='localhost', help="host, default localhost")
parser.add_argument("-d", "--diapasone", default=355, help="Full diapasone, default 355")

parser.add_argument("-t", "--threshold", default=50, help="Threshold for linear move")
parser.add_argument("--linear-threshold", help="Threshold for linear move")
parser.add_argument("--angular-threshold", help="Threshold for angular move")

parser.add_argument("--linear-scale", default=1.5, help="Scale factor for linear move")
parser.add_argument("--angular-scale", default=1.0, help="Scale factor for linear move")

args = parser.parse_args()

QUITE = args.quite
RECONNECT = args.reconnect
HOST = args.host
PORT = args.port
FULL_SCALE = float(args.diapasone)
treshold = args.threshold
LINEAR_DEAD_ZONE = args.linear_threshold if args.linear_threshold else treshold
ANGULAR_DEAD_ZONE = args.angular_threshold if args.angular_threshold else treshold
LINEAR_SCALE = float(args.linear_scale)
ANGULAR_SCALE = float(args.angular_scale)


def main():
    spnav_open()

    s = open_scoket(host=HOST, port=PORT, reconnect=RECONNECT)

    def process_spnav_evnt(event):
        norm_event = normalize(event)
        if norm_event:
            json_string = json.dumps(norm_event)
            send(json_string, sock=s)

    def poll_loop():
        event = spnav_poll_event()
        if event is not None:
            if event.ev_type == SPNAV_EVENT_MOTION:
                process_spnav_evnt(event)

    try:
        while True:
            poll_loop()
    except KeyboardInterrupt:
        print '\nQuitting...'
    except socket.error, err:
        print "Connection error: " + err.strerror
    finally:
        spnav_close()
        socket_close(s)


def send(msg, sock=None):
    data = msg + "\n"
    sock.send(data.encode('utf-8'))
    if not QUITE:
        print msg


def socket_close(sock):
    sock.close()


def normalize(event):

    lx = event.translation[2] / FULL_SCALE * LINEAR_SCALE
    ly = -event.translation[0] / FULL_SCALE * LINEAR_SCALE
    lz = event.translation[1] / FULL_SCALE * LINEAR_SCALE

    rx = event.rotation[2] / FULL_SCALE * ANGULAR_SCALE
    ry = -event.rotation[0] / FULL_SCALE * ANGULAR_SCALE
    rz = event.rotation[1] / FULL_SCALE * ANGULAR_SCALE

    linear_dead = fabs(event.translation[0]) < LINEAR_DEAD_ZONE
    linear_dead = linear_dead and fabs(event.translation[1]) < LINEAR_DEAD_ZONE
    linear_dead = linear_dead and fabs(event.translation[2]) < LINEAR_DEAD_ZONE

    angular_dead = fabs(event.rotation[0]) < ANGULAR_DEAD_ZONE
    angular_dead = angular_dead and fabs(event.rotation[1]) < ANGULAR_DEAD_ZONE
    angular_dead = angular_dead and fabs(event.rotation[2]) < ANGULAR_DEAD_ZONE

    if linear_dead and angular_dead:
        return None

    result = {
        'type': 'motion',
        'trans': [lx, ly, lz],
        'rot': [rx, ry, rz]
    }

    return result


def open_scoket(host="localhost", port=6564, reconnect=True):
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    while True:
        try:
            try:
                print "Connecting to {} {}".format(host, port)
                soc.connect((host, port))
                break
            except IOError, e:
                if reconnect:
                    print "Failed to open connection, will retry in 2 seconds"
                    time.sleep(2)
                else:
                    print e
                    quit()
        except KeyboardInterrupt:
            quit()

    return soc


if __name__ == '__main__':
    main()
