#! /usr/bin/env python

import json
import socket

from spnav import spnav_open, spnav_poll_event, spnav_close
from spnav import SPNAV_EVENT_MOTION

def main():
    spnav_open()
    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                if event.ev_type == SPNAV_EVENT_MOTION:
                    json_string = asJson(event)
                    send(json_string)
    except KeyboardInterrupt:
        print '\nQuitting...'
    finally:
        spnav_close()

def send(msg):
    send_socket = None
    try:
        send_socket = open_scoket()
        send_socket.send(msg.encode())
        print msg
    except socket.error, err:
        print "Connection error: " + err.strerror
    finally:
        if send_socket:
            send_socket.close()

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
