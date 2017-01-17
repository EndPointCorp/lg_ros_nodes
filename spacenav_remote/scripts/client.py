#! /usr/bin/env python

import json
import socket

from spnav import spnav_open, spnav_poll_event, spnav_close
from spnav import SPNAV_EVENT_MOTION

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
    sock.send(msg)

def socket_close(sock):
    sock.close()

def asJson(event):
    result = {'type': 'motion', 'trans': event.translation, 'rot': event.rotation}
    return json.dumps(result)

def open_scoket(host="localhost", port=6564):
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    soc.connect((host, port))
    return soc

if __name__ == '__main__':
    main()
