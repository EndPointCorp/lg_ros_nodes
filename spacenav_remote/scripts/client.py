#! /usr/bin/env python

import json
import socket
from math import fabs

from spnav import spnav_open, spnav_poll_event, spnav_close
from spnav import SPNAV_EVENT_MOTION

FULL_SCALE = 512

def main():
    spnav_open()
    s = open_scoket()

    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                if event.ev_type == SPNAV_EVENT_MOTION:
                    json_string = asJson(event)
                    send(json_string, sock=s)
    except KeyboardInterrupt:
        print '\nQuitting...'
    except socket.error, err:
        print "Connection error: " + err.strerror
    finally:
        spnav_close()
        socket_close(s)

def send(msg, sock=None):
    data = msg + "\n"
    sock.send(data)
    print msg

def socket_close(sock):
    sock.close()

def asJson(event):

    lx = event.translation[2] / FULL_SCALE
    ly = -event.translation[0] / FULL_SCALE
    lz = event.translation[1] / FULL_SCALE

    rx = event.rotation[2] / FULL_SCALE
    ry = -event.rotation[0] / FULL_SCALE
    rz = event.rotation[1] / FULL_SCALE

    result = {
        'type' : 'motion',
        'trans': [lx, ly, lz],
        'rot'  : [rx, ry, rz]
    }

    return json.dumps(result)

def open_scoket(host="localhost", port=6564):
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    soc.connect((host, port))
    return soc

if __name__ == '__main__':
    main()
